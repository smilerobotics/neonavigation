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

#include <gtest/gtest.h>

#include <list>
#include <string>
#include <unordered_map>
#include <utility>

#include <nav_msgs/Path.h>
#include <planner_cspace/planner_3d/path_interpolator.h>
#include <planner_cspace/planner_3d/start_pose_predictor.h>

namespace planner_cspace
{
namespace planner_3d
{
class StartPosePredictorTester : public ::testing::Test
{
protected:
  void SetUp() final
  {
    config_ =
        {
            .prediction_sec_ = 0.5,
            .switch_back_prediction_sec_ = 2.0,
            .dist_stop_ = 0.1,
            .lin_vel_ = 1.5,
            .ang_vel_ = M_PI / 2,
        };
    predictor_.setConfig(config_);

    map_info_.width = 11;
    map_info_.height = 11;
    map_info_.angle = 4;
    map_info_.origin.position.x = -5.5;
    map_info_.origin.position.y = -5.5;
    map_info_.origin.orientation.w = 1.0;
    map_info_.linear_resolution = 1.0;
    map_info_.angular_resolution = 2 * M_PI / map_info_.angle;
    interpolator_.reset(map_info_.linear_resolution, 1);

    const GridAstar<3, 2>::Vec size3d(
        static_cast<int>(map_info_.width),
        static_cast<int>(map_info_.height),
        static_cast<int>(map_info_.angle));
    cm_.reset(size3d);
    for (int a = 0; a < map_info_.angle; ++a)
    {
      for (int y = 0; y < map_info_.height; ++y)
      {
        for (int x = 0; x < map_info_.width; ++x)
        {
          cm_[GridAstar<3, 2>::Vec(x, y, a)] = 0;
        }
      }
    }

    const std::list<StartPosePredictor::Astar::Vec> path_grid_straight =
        {
            StartPosePredictor::Astar::Vec(5, 5, 0),  // Start is (0.0, 0.0, 0.0)
            StartPosePredictor::Astar::Vec(6, 5, 0),
            StartPosePredictor::Astar::Vec(7, 5, 0),  // Moves 2 meters ahead
        };
    paths_["straight"] = std::make_pair(path_grid_straight, getMetricPath(path_grid_straight));

    const std::list<StartPosePredictor::Astar::Vec> path_grid_in_place_turn =
        {
            StartPosePredictor::Astar::Vec(5, 5, 0),  // Start is (0.0, 0.0, 0.0)
            StartPosePredictor::Astar::Vec(6, 5, 0),
            StartPosePredictor::Astar::Vec(7, 5, 0),
            StartPosePredictor::Astar::Vec(8, 5, 0),  // Turn at (3.0, 0.0, 0.0)
            StartPosePredictor::Astar::Vec(8, 5, 1),
            StartPosePredictor::Astar::Vec(8, 6, 1),
            StartPosePredictor::Astar::Vec(8, 7, 1),  // Turn at (3.0, 2.0, pi/2)
            StartPosePredictor::Astar::Vec(8, 7, 2),  // Goal is (3.0, 2.0, pi)
        };
    paths_["in_place_turn"] = std::make_pair(path_grid_in_place_turn, getMetricPath(path_grid_in_place_turn));

    const std::list<StartPosePredictor::Astar::Vec> path_grid_switch_back =
        {
            StartPosePredictor::Astar::Vec(5, 5, 0),  // Start is (0.0, 0.0, 0.0)
            StartPosePredictor::Astar::Vec(6, 5, 0),
            StartPosePredictor::Astar::Vec(7, 5, 0),
            StartPosePredictor::Astar::Vec(8, 6, 1),   // Curv from (2.0, 0.0, 0.0) to (3.0, 1.0, pi/2)
            StartPosePredictor::Astar::Vec(9, 5, 2),   // Curv from (3.0, 1.0, 0.0) to (4.0, 0.0, pi)
            StartPosePredictor::Astar::Vec(10, 5, 2),  // goal is (5.0, 0.0, pi)
        };
    paths_["switch_back"] = std::make_pair(path_grid_switch_back, getMetricPath(path_grid_switch_back));
  }

  nav_msgs::Path getMetricPath(const std::list<GridAstar<3, 2>::Vec>& path_grid) const
  {
    const auto path_interpolated = interpolator_.interpolate(path_grid, 0.1, 10);

    nav_msgs::Path ret;
    ret.header.frame_id = "map";
    for (const auto& pose_grid : path_interpolated)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      float x, y, yaw;
      grid_metric_converter::grid2Metric(map_info_, pose_grid[0], pose_grid[1], pose_grid[2], x, y, yaw);
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.z = sin(yaw / 2);
      pose.pose.orientation.w = cos(yaw / 2);
      ret.poses.push_back(pose);
    }
    return ret;
  }

  geometry_msgs::Pose getPose(const double x, const double y, const double yaw) const
  {
    geometry_msgs::Pose ret;
    ret.position.x = x;
    ret.position.y = y;
    ret.orientation.z = sin(yaw / 2);
    ret.orientation.w = cos(yaw / 2);
    return ret;
  }

  StartPosePredictor predictor_;
  StartPosePredictor::Config config_;
  GridAstar<3, 2>::Gridmap<char, 0x40> cm_;
  costmap_cspace_msgs::MapMetaData3D map_info_;
  PathInterpolator interpolator_;
  std::unordered_map<std::string, std::pair<std::list<StartPosePredictor::Astar::Vec>, nav_msgs::Path>> paths_;
};

TEST_F(StartPosePredictorTester, Failures)
{
  StartPosePredictor::Astar::Vec start_grid;
  // Empty path
  EXPECT_EQ(
      predictor_.process(getPose(0.0, 0.0, 0.0), getPose(0.0, 0.0, 0.0), cm_, map_info_, nav_msgs::Path(), start_grid),
      StartPosePredictor::Result::FAILED);

  // Far from path
  EXPECT_EQ(predictor_.process(getPose(0.0, 0.5, 0.0), getPose(2.0, 0.0, 0.0), cm_, map_info_,
                               paths_["straight"].second, start_grid),
            StartPosePredictor::Result::FAILED);
}

TEST_F(StartPosePredictorTester, Normal)
{
  StartPosePredictor::Astar::Vec start_grid;
  EXPECT_EQ(predictor_.process(getPose(0.05, 0.05, 0.0), getPose(2.0, 0.0, 0.0), cm_, map_info_,
                               paths_["straight"].second, start_grid),
            StartPosePredictor::Result::OK);
  EXPECT_EQ(start_grid[0], 6);
  EXPECT_EQ(start_grid[1], 5);
  EXPECT_EQ(start_grid[2], 0);
  const auto preserved_path = predictor_.getPreservedPath();
  ASSERT_EQ(preserved_path.poses.size(), 9);
  for (size_t i = 0; i < preserved_path.poses.size(); ++i)
  {
    const auto& path_pose = preserved_path.poses[i].pose;
    EXPECT_NEAR(path_pose.position.x, 0.1f + 0.1f * i, 1.0e-6);
    EXPECT_NEAR(path_pose.position.y, 0.0, 1.0e-6);
    EXPECT_NEAR(tf2::getYaw(path_pose.orientation), 0.0, 1.0e-6);
  }
  EXPECT_NEAR(predictor_.getPreservedPathLength(), 0.9, 1.0e-6);

  cm_[GridAstar<3, 2>::Vec(6, 5, 0)] = 100;
  EXPECT_EQ(predictor_.process(getPose(0.05, 0.05, 0.0), getPose(2.0, 0.0, 0.0), cm_, map_info_,
                               paths_["straight"].second, start_grid),
            StartPosePredictor::Result::START_OCCUPIED);
  EXPECT_TRUE(predictor_.getPreservedPath().poses.empty());
  EXPECT_EQ(predictor_.getPreservedPathLength(), 0.0);
}

TEST_F(StartPosePredictorTester, SwitchBack)
{
  {
    StartPosePredictor::Astar::Vec start_grid;
    const auto result = predictor_.process(getPose(0.25, 0.05, 0.0), getPose(2.0, 2.0, M_PI / 2), cm_, map_info_,
                                           paths_["switch_back"].second, start_grid);

    // The robot will not reach the switch back within 2.0 seconds, and the start grid will be on the first line.
    EXPECT_EQ(start_grid[0], 7);
    EXPECT_EQ(start_grid[1], 5);
    EXPECT_EQ(start_grid[2], 0);
    EXPECT_EQ(result, StartPosePredictor::Result::OK);

    const auto preserved_path = predictor_.getPreservedPath();
    ASSERT_EQ(preserved_path.poses.size(), 17);
    for (size_t i = 0; i < preserved_path.poses.size(); ++i)
    {
      const auto& path_pose = preserved_path.poses[i].pose;
      EXPECT_NEAR(path_pose.position.x, 0.3f + 0.1f * i, 1.0e-6);
      EXPECT_NEAR(path_pose.position.y, 0.0, 1.0e-6);
      EXPECT_NEAR(tf2::getYaw(path_pose.orientation), 0.0, 1.0e-6);
    }
  }

  {
    StartPosePredictor::Astar::Vec start_grid;
    const auto result = predictor_.process(getPose(0.45, 0.05, 0.0), getPose(2.0, 2.0, M_PI / 2), cm_, map_info_,
                                           paths_["switch_back"].second, start_grid);

    // The robot will reach the switch back within 2.0 seconds, and the start grid is the switch back point.
    EXPECT_EQ(start_grid[0], 8);
    EXPECT_EQ(start_grid[1], 6);
    EXPECT_EQ(start_grid[2], 1);
    EXPECT_EQ(result, StartPosePredictor::Result::OK);

    const auto preserved_path = predictor_.getPreservedPath();
    // A part of the first forward, the second forward, and the first turn.
    EXPECT_EQ(preserved_path.poses.size(), 6 + 10 + 14);
    EXPECT_NEAR(preserved_path.poses.front().pose.position.x, 0.5, 1.0e-6);
    EXPECT_NEAR(preserved_path.poses.front().pose.position.y, 0.0, 1.0e-6);
    EXPECT_NEAR(tf2::getYaw(preserved_path.poses.front().pose.orientation), 0.0, 1.0e-6);
    EXPECT_NEAR(preserved_path.poses.back().pose.position.x, 3.0, 1.0e-6);
    EXPECT_NEAR(preserved_path.poses.back().pose.position.y, 1.0, 1.0e-6);
    EXPECT_NEAR(tf2::getYaw(preserved_path.poses.back().pose.orientation), M_PI / 2, 1.0e-6);
  }
}

}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
