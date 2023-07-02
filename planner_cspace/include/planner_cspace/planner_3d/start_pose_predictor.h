/*
 * Copyright (c) 2023, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PLANNER_CSPACE_PLANNER_3D_START_POSE_PREDICTOR_H
#define PLANNER_CSPACE_PLANNER_3D_START_POSE_PREDICTOR_H

#include <vector>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/grid_metric_converter.h>

#include <trajectory_tracker/path2d.h>

namespace planner_cspace
{
namespace planner_3d
{

class StartPosePredictor
{
public:
  using Astar = GridAstar<3, 2>;

  struct Config
  {
    double prediction_sec_;
    double switch_back_prediction_sec_;
    double dist_stop_;
    double lin_vel_;
    double ang_vel_;
  };

  void setConfig(const Config& config)
  {
    config_ = config;
  }

  enum class Result
  {
    OK,
    FAILED,
    START_OCCUPIED,
    NO_NEED_TO_REPLAN,
  };

  Result process(const geometry_msgs::Pose& start_metric,
                 const geometry_msgs::Pose& goal_metric,
                 const GridAstar<3, 2>::Gridmap<char, 0x40>& cm,
                 const costmap_cspace_msgs::MapMetaData3D& map_info,
                 const nav_msgs::Path& previous_path,
                 Astar::Vec& start_grid)
  {
    preserved_path_.header = previous_path.header;
    preserved_path_.poses.clear();
    preserved_path_length_ = 0.0;
    previous_path_2d_.fromMsg(previous_path.poses);
    map_info_ = map_info;
    for (size_t i = 0; i < previous_path_2d_.size(); ++i)
    {
      const auto& p = previous_path_2d_[i];
      ROS_DEBUG("#%lu: x: %f, y: %f, yaw: %f, isCenter: %d", i, p.pos_.x(), p.pos_.y(), p.yaw_, isGridCenter(p));
    }
    if (!removeAlreadyPassed(start_metric) || previous_path_2d_.empty())
    {
      return Result::FAILED;
    }

    double angle_diff = std::abs(previous_path_2d_.begin()->yaw_ - tf2::getYaw(start_metric.orientation));
    if (angle_diff > M_PI)  // Normalize angle
    {
      angle_diff = 2 * M_PI - angle_diff;
    }
    const double initial_eta = std::abs(angle_diff) / config_.ang_vel_;
    const std::vector<double> etas = previous_path_2d_.getETAs(
        previous_path_2d_.begin(), previous_path_2d_.end(), config_.lin_vel_, config_.ang_vel_, initial_eta);

    const auto local_goal_it = getSwitchBack(etas);
    if (local_goal_it != previous_path_2d_.end())
    {
      return buildResults(local_goal_it, start_metric, cm, start_grid);
    }

    const auto expected_pose_it = getExpectedPose(etas);
    if (expected_pose_it != previous_path_2d_.end())
    {
      return buildResults(expected_pose_it, start_metric, cm, start_grid);
    }

    ROS_DEBUG("The robot will reach the goal in %f sec. No need to replan.", etas.back());
    // return Result::NO_NEED_TO_REPLAN;
    return buildResults(previous_path_2d_.end() - 1, start_metric, cm, start_grid);
  }
  const nav_msgs::Path& getPreservedPath() const
  {
    return preserved_path_;
  }
  const double getPreservedPathLength() const
  {
    return preserved_path_length_;
  }
  void clear()
  {
    previous_path_2d_.clear();
    preserved_path_.poses.clear();
    preserved_path_length_ = 0.0;
  }

private:
  bool removeAlreadyPassed(const geometry_msgs::Pose& start_metric)
  {
    const Eigen::Vector2d robot_pose_2d(start_metric.position.x, start_metric.position.y);
    double dist_err;
    trajectory_tracker::Path2D::ConstIterator it_nearest;
    std::tie(it_nearest, dist_err) =
        previous_path_2d_.findNearestDistance(previous_path_2d_.begin(), previous_path_2d_.end(), robot_pose_2d);
    if (it_nearest == previous_path_2d_.end())
    {
      // This happens only when previous_path_2d_ was empty.
      ROS_DEBUG("Failed to get nearest point.");
      return false;
    }
    if (dist_err > config_.dist_stop_)
    {
      ROS_WARN("The robot is too far from path. (%f)", dist_err);
      return false;
    }
    const size_t nearest_pos = it_nearest - previous_path_2d_.begin();
    ROS_DEBUG("Nearest pose in previous path: (%f, %f, %f), dist: %f, index: %lu, remaining poses num: %lu",
              it_nearest->pos_.x(), it_nearest->pos_.y(), it_nearest->yaw_, dist_err, nearest_pos,
              previous_path_2d_.size() - nearest_pos);
    previous_path_2d_.erase(previous_path_2d_.begin(), it_nearest);
    return true;
  }

  Result buildResults(const trajectory_tracker::Path2D::ConstIterator& expected_start_pose_it,
                      const geometry_msgs::Pose& start_metric, const GridAstar<3, 2>::Gridmap<char, 0x40>& cm,
                      Astar::Vec& start_grid)
  {
    for (auto it = previous_path_2d_.begin(); it != expected_start_pose_it; ++it)
    {
      Astar::Vec grid;
      grid_metric_converter::metric2Grid(map_info_, grid[0], grid[1], grid[2], it->pos_.x(), it->pos_.y(), it->yaw_);
      grid.cycleUnsigned(map_info_.angle);
      if (cm[grid] == 100)
      {
        ROS_WARN("The robot might collide with an obstacle during the next planning. An empty path is published.");
        clear();
        return Result::START_OCCUPIED;
      }
    }

    const auto expected_start_pose = *expected_start_pose_it;

    grid_metric_converter::metric2Grid(
        map_info_, start_grid[0], start_grid[1], start_grid[2],
        expected_start_pose_it->pos_.x(), expected_start_pose_it->pos_.y(), expected_start_pose_it->yaw_);
    start_grid.cycleUnsigned(map_info_.angle);
    const size_t previous_path_size = previous_path_2d_.size();

    previous_path_2d_.erase(expected_start_pose_it + 1, previous_path_2d_.end());
    ROS_DEBUG("Start grid: (%d, %d, %d), path size: %lu -> %lu",
              start_grid[0], start_grid[1], start_grid[2], previous_path_size, previous_path_2d_.size());
    if (!previous_path_2d_.empty())
    {
      preserved_path_length_ = previous_path_2d_.length();
      previous_path_2d_.toMsg(preserved_path_);
      preserved_path_.poses.pop_back();
    }
    return Result::OK;
  }

  bool isGridCenter(const trajectory_tracker::Pose2D& pose) const
  {
    Astar::Vecf grid;
    grid_metric_converter::metric2Grid(map_info_, grid[0], grid[1], grid[2], pose.pos_.x(), pose.pos_.y(), pose.yaw_);
    const float x_diff = std::abs(grid[0] - 0.5 - std::round(grid[0] - 0.5));
    const float y_diff = std::abs(grid[1] - 0.5 - std::round(grid[1] - 0.5));
    const float r_diff = std::abs(grid[2] - std::round(grid[2]));
    return (x_diff < 1.0e-3) && (y_diff < 1.0e-3) && (r_diff < 1.0e-3);
  }
  bool isPathColliding(const trajectory_tracker::Path2D::ConstIterator& begin,
                       const trajectory_tracker::Path2D::ConstIterator& end,
                       const GridAstar<3, 2>::Gridmap<char, 0x40>& cm)
  {
    for (auto it = begin; it != end; ++it)
    {
      Astar::Vec grid;
      grid_metric_converter::metric2Grid(map_info_, grid[0], grid[1], grid[2], it->pos_.x(), it->pos_.y(), it->yaw_);
      grid.cycleUnsigned(map_info_.angle);
      if (cm[grid] == 100)
      {
        ROS_DEBUG("Path colliding at (%f, %f, %f)", it->pos_.x(), it->pos_.y(), it->yaw_);
        return true;
      }
    }
    return false;
  }


  trajectory_tracker::Path2D::ConstIterator getSwitchBack(const std::vector<double>& etas) const
  {
    const auto local_goal_its = previous_path_2d_.enumerateLocalGoals(
        previous_path_2d_.begin(), previous_path_2d_.end(), true, false);
    if (local_goal_its.empty())
    {
      return previous_path_2d_.end();
    }

    for (const auto& local_goal : local_goal_its)
    {
      // Note that Path2D::findNearest() never returns the begin iterator and we can safely subtract 1.
      const size_t local_goal_index = local_goal - previous_path_2d_.begin() - 1;
      const double local_goal_eta = etas[local_goal_index];
      ROS_DEBUG("Switch back index: %lu, eta: %f", local_goal_index, local_goal_eta);
      if (isGridCenter(previous_path_2d_[local_goal_index]))
      {
        if ((config_.prediction_sec_ < local_goal_eta) &&
            (local_goal_eta < config_.switch_back_prediction_sec_))
        {
          ROS_INFO("The robot will reach a switch back point in %f sec.", local_goal_eta);
          // Actually returning the next of the switch back point.
          return local_goal;
        }
      }
      else
      {
        ROS_INFO("Switch back point is not grid center. (%f, %f, %f) -> (%f, %f, %f)",
                 previous_path_2d_[local_goal_index].pos_.x(), previous_path_2d_[local_goal_index].pos_.y(),
                 previous_path_2d_[local_goal_index].yaw_, previous_path_2d_[local_goal_index + 1].pos_.x(),
                 previous_path_2d_[local_goal_index + 1].pos_.y(), previous_path_2d_[local_goal_index + 1].yaw_);
      }
    }
    return previous_path_2d_.end();
  }

  trajectory_tracker::Path2D::ConstIterator getExpectedPose(const std::vector<double>& etas) const
  {
    for (size_t i = 0; i < previous_path_2d_.size() - 1; ++i)
    {
      const double eta = etas[i];
      const auto p = previous_path_2d_[i];
      ROS_DEBUG("Expected: #%lu: x: %f, y: %f, yaw: %f, eta: %f", i, p.pos_.x(), p.pos_.y(), p.yaw_, eta);
      if (eta > config_.prediction_sec_)
      {
        if (isGridCenter(previous_path_2d_[i]))
        {
          ROS_DEBUG("The robot will reach a grid center in %f sec.", eta);
          return previous_path_2d_.begin() + i;
        }
      }
    }
    return previous_path_2d_.end();
  }
  Config config_;
  trajectory_tracker::Path2D previous_path_2d_;
  costmap_cspace_msgs::MapMetaData3D map_info_;

  nav_msgs::Path preserved_path_;
  double preserved_path_length_;
};

}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_START_POSE_PREDICTOR_H
