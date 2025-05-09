/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
   Original idea of the implemented control scheme was published on:
   S. Iida, S. Yuta, "Vehicle command system and trajectory control for
   autonomous mobile robots," in Proceedings of the 1991 IEEE/RSJ
   International Workshop on Intelligent Robots and Systems (IROS),
   1991, pp. 212-217.
 */

#include <algorithm>
#include <condition_variable>
#include <cmath>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>
#include <trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp>
#include <neonavigation_common/neonavigation_node.hpp>

#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>
#include <trajectory_tracker/basic_control.h>

#include <trajectory_tracker/tracker_node.hpp>

namespace trajectory_tracker
{

TrackerNode::TrackerNode(const std::string& name, const rclcpp::NodeOptions& options)
  : neonavigation_common::NeonavigationNode(name, options)
  , tfbuf_(get_clock())
  , tfl_(tfbuf_)
  , is_path_updated_(false)
{
}

void TrackerNode::computeControlNormal()
{
  computeControl<Action>(action_server_normal_);
}
void TrackerNode::computeControlWithVelocity()
{
  computeControl<ActionWithVelocity>(action_server_with_velocity_);
}

void TrackerNode::initialize()
{
  declare_dynamic_parameter("look_forward", &look_forward_, 0.5);
  declare_dynamic_parameter("curv_forward", &curv_forward_, 0.5);
  declare_dynamic_parameter("k_dist", &k_[0], 1.0);
  declare_dynamic_parameter("k_ang", &k_[1], 1.0);
  declare_dynamic_parameter("k_avel", &k_[2], 1.0);
  declare_dynamic_parameter("gain_at_vel", &gain_at_vel_, 0.0);
  declare_dynamic_parameter("dist_lim", &d_lim_, 0.5);
  declare_dynamic_parameter("dist_stop", &d_stop_, 2.0);
  declare_dynamic_parameter("rotate_ang", &rotate_ang_, 0.78539816339);
  declare_dynamic_parameter("max_vel", &vel_[0], 0.5);
  declare_dynamic_parameter("max_angvel", &vel_[1], 1.0);
  declare_dynamic_parameter("max_acc", &acc_[0], 1.0);
  declare_dynamic_parameter("max_angacc", &acc_[1], 2.0);
  declare_dynamic_parameter("acc_toc_factor", &acc_toc_factor_, 0.9);
  declare_dynamic_parameter("angacc_toc_factor", &angacc_toc_factor_, 0.9);
  declare_dynamic_parameter("path_step", &path_step_, 1l);
  declare_dynamic_parameter("goal_tolerance_dist", &goal_tolerance_dist_, 0.2);
  declare_dynamic_parameter("goal_tolerance_ang", &goal_tolerance_ang_, 0.1);
  declare_dynamic_parameter("stop_tolerance_dist", &stop_tolerance_dist_, 0.1);
  declare_dynamic_parameter("stop_tolerance_ang", &stop_tolerance_ang_, 0.05);
  declare_dynamic_parameter("no_position_control_dist", &no_pos_cntl_dist_, 0.0);
  declare_dynamic_parameter("min_tracking_path", &min_track_path_, 0.0);
  declare_dynamic_parameter("allow_backward", &allow_backward_, true);
  declare_dynamic_parameter("limit_vel_by_avel", &limit_vel_by_avel_, false);
  declare_dynamic_parameter("check_old_path", &check_old_path_, false);
  declare_dynamic_parameter("epsilon", &epsilon_, 0.001);
  declare_dynamic_parameter("use_time_optimal_control", &use_time_optimal_control_, true);
  declare_dynamic_parameter("time_optimal_control_future_gain", &time_optimal_control_future_gain_, 1.5);
  declare_dynamic_parameter("k_ang_rotation", &k_ang_rotation_, 1.0);
  declare_dynamic_parameter("k_avel_rotation", &k_avel_rotation_, 1.0);
  declare_dynamic_parameter("goal_tolerance_lin_vel", &goal_tolerance_lin_vel_, 0.0);
  declare_dynamic_parameter("goal_tolerance_ang_vel", &goal_tolerance_ang_vel_, 0.0);
  declare_dynamic_parameter("frame_robot", &frame_robot_, std::string("base_link"));
  declare_dynamic_parameter("frame_odom", &frame_odom_, std::string("odom"));
  declare_dynamic_parameter("hz", &hz_, 50.0);
  declare_dynamic_parameter("predict_odom", &predict_odom_, true);
  declare_dynamic_parameter("max_dt", &max_dt_, 0.1);
  declare_dynamic_parameter("odom_timeout_sec", &odom_timeout_sec_, 0.1);
  declare_dynamic_parameter("unable_to_follow_path_threshold", &unable_to_follow_path_threshold_, 5l);
  declare_dynamic_parameter("tracking_search_range", &tracking_search_range_, 1.0);
  declare_dynamic_parameter("initial_tracking_search_range", &initial_tracking_search_range_, 0.0);
  declare_dynamic_parameter("keep_last_rotation", &keep_last_rotation_, false);
  double action_server_wait_duration_sec = 0.1;
  declare_dynamic_parameter("action_server_wait_duration_sec",
                            &action_server_wait_duration_sec, action_server_wait_duration_sec);
  action_server_wait_duration_ = std::chrono::duration<double>(action_server_wait_duration_sec);
  is_robot_rotating_on_last_ = false;

  onDynamicParameterUpdated({});

  const bool use_action_server = declare_parameter("use_action_server", false);
  if (use_action_server)
  {
    action_server_normal_ = std::make_shared<ActionServer>(shared_from_this(), "follow_path",
                                                           std::bind(&TrackerNode::computeControlNormal, this), nullptr,
                                                           std::chrono::milliseconds(500), false);
    action_server_normal_->activate();
  }
  const bool use_action_server_with_velocity = declare_parameter("use_action_server_with_velocity", false);
  if (use_action_server_with_velocity)
  {
    action_server_with_velocity_ = std::make_shared<ActionServerWithVelocity>(
        shared_from_this(), "follow_path_with_velocity", std::bind(&TrackerNode::computeControlWithVelocity, this),
        nullptr, std::chrono::milliseconds(500), false);
    action_server_with_velocity_->activate();
  }
  if (use_action_server || use_action_server_with_velocity)
  {
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TrackerNode::cbOdometry, this, std::placeholders::_1));
    pub_remaining_path_ = create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    pub_tracking_path_ =
        create_publisher<nav_msgs::msg::Path>("tracking_path", rclcpp::QoS(1).transient_local().reliable());
  }
  else
  {
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        "path", 2, std::bind(&TrackerNode::cbPath<nav_msgs::msg::Path>, this, std::placeholders::_1));
    sub_path_velocity_ = create_subscription<trajectory_tracker_msgs::msg::PathWithVelocity>(
        "path_velocity", 2,
        std::bind(&TrackerNode::cbPath<trajectory_tracker_msgs::msg::PathWithVelocity>, this, std::placeholders::_1));
    if (declare_parameter("use_odom", false))
    {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&TrackerNode::cbOdometry, this, std::placeholders::_1));
    }
    else
    {
      timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / hz_),
                                    std::bind(&TrackerNode::cbTimer, this));
    }
  }
  sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      "speed", 20, std::bind(&TrackerNode::cbSpeed, this, std::placeholders::_1));

  pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pub_status_ = create_publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>("~/status", 10);
  pub_tracking_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tracking", 10);

  rclcpp::on_shutdown(
      [this]()
      {
        if (action_server_normal_)
        {
          action_server_normal_->deactivate();
        }
        if (action_server_with_velocity_)
        {
          action_server_with_velocity_->deactivate();
        }
        publishZeroVelocity();
      });
}

void TrackerNode::onDynamicParameterUpdated(const std::vector<rclcpp::Parameter>&)
{
  acc_toc_[0] = acc_[0] * acc_toc_factor_;
  acc_toc_[1] = acc_[1] * angacc_toc_factor_;
}

void TrackerNode::cbSpeed(const std_msgs::msg::Float32& msg)
{
  vel_[0] = msg.data;
}

template <typename MSG_TYPE>
bool TrackerNode::shouldKeepRotation(const MSG_TYPE& msg) const
{
  if (is_robot_rotating_on_last_ && msg.poses.size() > 0 && path_.size() > 0)
  {
    const auto& last_pose = msg.poses.back();
    const auto& last_path_pose = path_.back();
    const double yaw_diff = std::abs(tf2::getYaw(last_pose.pose.orientation) - last_path_pose.yaw_);
    const double dist_diff = std::hypot(last_pose.pose.position.x - last_path_pose.pos_.x(),
                                        last_pose.pose.position.y - last_path_pose.pos_.y());
    if (yaw_diff < goal_tolerance_dist_ && dist_diff < goal_tolerance_ang_)
    {
      return true;
    }
    RCLCPP_DEBUG(get_logger(), "Goal updated. yaw_diff %0.3f, dist_diff %0.3f", yaw_diff, dist_diff);
  }
  return false;
}

void TrackerNode::publishTrackingPath(const nav_msgs::msg::Path& path)
{
  received_path_ = path;
  if (pub_tracking_path_)
  {
    pub_tracking_path_->publish(received_path_);
  }
}

void TrackerNode::publishTrackingPath(const trajectory_tracker_msgs::msg::PathWithVelocity& path_with_velocity)
{
  nav_msgs::msg::Path path;
  path.header = path_with_velocity.header;
  path.poses.reserve(path_with_velocity.poses.size());
  for (const auto& pose_with_velocity : path_with_velocity.poses)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_with_velocity.header;
    pose.pose = pose_with_velocity.pose;
    path.poses.emplace_back(pose);
  }
  publishTrackingPath(path);
}

template <typename MSG_TYPE>
void TrackerNode::cbPath(const MSG_TYPE& msg)
{
  path_header_ = msg.header;
  is_path_updated_ = true;
  path_step_done_ = 0;

  if (msg.poses.size() > 0)
  {
    const geometry_msgs::msg::Pose& last_pose = msg.poses.back().pose;
    RCLCPP_DEBUG(get_logger(), "Goal Pose: (%f, %f), yaw: %f", last_pose.position.x, last_pose.position.y,
                 tf2::getYaw(last_pose.orientation));
  }

  if (shouldKeepRotation(msg))
  {
    RCLCPP_INFO(get_logger(), "Robot is rotating on last pose and the goal is not updated. Keep the last rotation.");
    MSG_TYPE new_path_msg;
    new_path_msg.header = msg.header;
    new_path_msg.poses.push_back(msg.poses.back());
    path_.fromMsgWithIndices(new_path_msg, epsilon_, path_to_msg_indices_);
  }
  else
  {
    path_.fromMsgWithIndices(msg, epsilon_, path_to_msg_indices_);
    is_robot_rotating_on_last_ = false;
  }
  for (const auto& path_pose : path_)
  {
    if (std::isfinite(path_pose.velocity_) && path_pose.velocity_ < -0.0)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "path_velocity.velocity.x must be positive");
      path_.clear();
      return;
    }
  }
  publishTrackingPath(msg);
}

bool TrackerNode::isControlNeeded() const
{
  if (!action_server_normal_ && !action_server_with_velocity_)
  {
    // Normal mode: always running.
    return true;
  }

  if (action_server_normal_ && action_server_normal_->is_running())
  {
    // Normal action server is running.
    return true;
  }
  if (action_server_with_velocity_ && action_server_with_velocity_->is_running())
  {
    // With velocity action server is running.
    return true;
  }
  return false;
}

void TrackerNode::cbOdometry(const nav_msgs::msg::Odometry& odom)
{
  if (odom.header.frame_id != frame_odom_)
  {
    RCLCPP_WARN(get_logger(), "frame_odom is invalid. Update from \"%s\" to \"%s\"", frame_odom_.c_str(),
                odom.header.frame_id.c_str());
    frame_odom_ = odom.header.frame_id;
  }
  if (odom.child_frame_id != frame_robot_)
  {
    RCLCPP_WARN(get_logger(), "frame_robot is invalid. Update from \"%s\" to \"%s\"", frame_robot_.c_str(),
                odom.child_frame_id.c_str());
    frame_robot_ = odom.child_frame_id;
  }
  if (odom_timeout_sec_ != 0.0)
  {
    if (odom_timeout_timer_)
    {
      odom_timeout_timer_->cancel();
    }
    odom_timeout_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(odom_timeout_sec_),
                                               std::bind(&TrackerNode::cbOdomTimeout, this));
  }

  if ((prev_odom_stamp_.nanoseconds() != 0L) && (isControlNeeded()))
  {
    const rclcpp::Time odom_stamp(odom.header.stamp);
    const double dt = std::min(max_dt_, (odom_stamp - prev_odom_stamp_).seconds());
    nav_msgs::msg::Odometry odom_compensated = odom;
    Eigen::Vector3d prediction_offset(0, 0, 0);
    if (predict_odom_)
    {
      const double predict_dt = std::max(0.0, std::min(max_dt_, (now() - odom_stamp).seconds()));
      tf2::Transform trans;
      const tf2::Quaternion rotation(tf2::Vector3(0, 0, 1), odom.twist.twist.angular.z * predict_dt);
      const tf2::Vector3 translation(odom.twist.twist.linear.x * predict_dt, 0, 0);

      prediction_offset[0] = odom.twist.twist.linear.x * predict_dt;
      prediction_offset[2] = odom.twist.twist.angular.z * predict_dt;

      tf2::fromMsg(odom.pose.pose, trans);
      trans.setOrigin(trans.getOrigin() + tf2::Transform(trans.getRotation()) * translation);
      trans.setRotation(trans.getRotation() * rotation);
      tf2::toMsg(trans, odom_compensated.pose.pose);
    }
    tf2::Transform odom_to_robot;
    tf2::fromMsg(odom_compensated.pose.pose, odom_to_robot);
    const tf2::Stamped<tf2::Transform> robot_to_odom(odom_to_robot.inverse(), tf2_ros::fromMsg(odom_stamp),
                                                     odom.header.frame_id);

    control(robot_to_odom, prediction_offset, odom.twist.twist.linear.x, odom.twist.twist.angular.z, dt);
  }
  prev_odom_stamp_ = odom.header.stamp;
}

void TrackerNode::cbTimer()
{
  try
  {
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(tfbuf_.lookupTransform(frame_robot_, frame_odom_, rclcpp::Time(0)), transform);
    control(transform, Eigen::Vector3d(0, 0, 0), 0, 0, 1.0 / hz_);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF exception: %s", e.what());
    trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
    status.header.stamp = now();
    status.distance_remains = 0.0;
    status.angle_remains = 0.0;
    status.path_header = path_header_;
    status.last_passed_index = 0;
    status.distance_to_path = 0.0;
    status.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
    pub_status_->publish(status);
    return;
  }
}

void TrackerNode::cbOdomTimeout()
{
  const std::lock_guard<std::mutex> lock(action_server_mutex_);
  RCLCPP_WARN_STREAM(get_logger(), "Odometry timeout. Last odometry stamp: " << prev_odom_stamp_.seconds());
  v_lim_.clear();
  w_lim_.clear();
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  pub_vel_->publish(cmd_vel);

  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  status.header.stamp = now();
  status.distance_remains = 0.0;
  status.angle_remains = 0.0;
  status.path_header = path_header_;
  status.last_passed_index = 0;
  status.distance_to_path = 0.0;
  status.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
  pub_status_->publish(status);

  // One shot timer
  odom_timeout_timer_->cancel();
  odom_timeout_timer_.reset();
  action_server_feedback_cv_.notify_all();
}

void TrackerNode::control(const tf2::Stamped<tf2::Transform>& robot_to_odom, const Eigen::Vector3d& prediction_offset,
                          const double odom_linear_vel, const double odom_angular_vel, const double dt)
{
  const std::lock_guard<std::mutex> lock(action_server_mutex_);
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  status.header.stamp = now();
  status.path_header = path_header_;
  if (is_path_updated_)
  {
    // Call getTrackingResult to update path_step_done_.
    const TrackingResult initial_tracking_result =
        getTrackingResult(robot_to_odom, prediction_offset, odom_linear_vel, odom_angular_vel);
    path_step_done_ = initial_tracking_result.path_step_done;
    is_path_updated_ = false;
  }
  const TrackingResult tracking_result =
      getTrackingResult(robot_to_odom, prediction_offset, odom_linear_vel, odom_angular_vel);
  switch (tracking_result.status)
  {
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH:
    {
      v_lim_.clear();
      w_lim_.clear();
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      pub_vel_->publish(cmd_vel);
      break;
    }
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL:
    {
      is_robot_rotating_on_last_ = false;
    }
    default:
    {
      if (tracking_result.turning_in_place)
      {
        v_lim_.set(0.0, tracking_result.target_linear_vel, acc_[0], dt);

        if (use_time_optimal_control_)
        {
          const double expected_angle_remains =
              tracking_result.angle_remains + w_lim_.get() * dt * time_optimal_control_future_gain_;
          w_lim_.set(trajectory_tracker::timeOptimalControl(expected_angle_remains, acc_toc_[1]), vel_[1], acc_[1], dt);
        }
        else
        {
          const double wvel_increment =
              (-tracking_result.angle_remains * k_ang_rotation_ - w_lim_.get() * k_avel_rotation_) * dt;
          w_lim_.increment(wvel_increment, vel_[1], acc_[1], dt);
        }
        RCLCPP_DEBUG(get_logger(), "angular residual %0.3f, angular vel %0.3f", tracking_result.angle_remains,
                     w_lim_.get());
        if (keep_last_rotation_ && (tracking_result.distance_remains == 0.0))
        {
          RCLCPP_DEBUG(get_logger(), "Robot is rotating on last pose");
          is_robot_rotating_on_last_ = true;
        }
      }
      else
      {
        v_lim_.set(trajectory_tracker::timeOptimalControl(tracking_result.signed_local_distance, acc_toc_[0]),
                   tracking_result.target_linear_vel, acc_[0], dt);

        float wref = std::abs(v_lim_.get()) * tracking_result.tracking_point_curv;

        if (limit_vel_by_avel_ && std::abs(wref) > vel_[1])
        {
          v_lim_.set(std::copysign(1.0, v_lim_.get()) * std::abs(vel_[1] / tracking_result.tracking_point_curv),
                     tracking_result.target_linear_vel, acc_[0], dt);
          wref = std::copysign(1.0, wref) * vel_[1];
        }

        const double k_ang =
            (gain_at_vel_ == 0.0) ? (k_[1]) : (k_[1] * tracking_result.target_linear_vel / gain_at_vel_);
        const double dist_diff = tracking_result.distance_from_target;
        const double angle_diff = tracking_result.angle_remains;
        const double wvel_diff = w_lim_.get() - wref;
        w_lim_.increment(dt * (-dist_diff * k_[0] - angle_diff * k_ang - wvel_diff * k_[2]), vel_[1], acc_[1], dt);

        RCLCPP_DEBUG(get_logger(),
                     "distance residual %0.3f, angular residual %0.3f, ang vel residual %0.3f"
                     ", v_lim %0.3f, w_lim %0.3f signed_local_distance %0.3f, k_ang %0.3f",
                     dist_diff, angle_diff, wvel_diff, v_lim_.get(), w_lim_.get(),
                     tracking_result.signed_local_distance, k_ang);
      }
      if (std::abs(tracking_result.distance_remains) < stop_tolerance_dist_ &&
          std::abs(tracking_result.angle_remains) < stop_tolerance_ang_ &&
          std::abs(tracking_result.distance_remains_raw) < stop_tolerance_dist_ &&
          std::abs(tracking_result.angle_remains_raw) < stop_tolerance_ang_)
      {
        v_lim_.clear();
        w_lim_.clear();
      }
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = v_lim_.get();
      cmd_vel.angular.z = w_lim_.get();
      pub_vel_->publish(cmd_vel);
      path_step_done_ = tracking_result.path_step_done;
      break;
    }
  }
  status.status = tracking_result.status;
  status.distance_remains = tracking_result.distance_remains;
  status.angle_remains = tracking_result.angle_remains;
  if (path_to_msg_indices_.empty())
  {
    status.last_passed_index = 0;
  }
  else
  {
    if (tracking_result.path_step_done < path_to_msg_indices_.size())
    {
      status.last_passed_index = path_to_msg_indices_[tracking_result.path_step_done];
    }
    else
    {
      status.last_passed_index = path_to_msg_indices_.back();
    }
  }
  status.distance_to_path = tracking_result.distance_from_target;
  pub_status_->publish(status);
  latest_status_ = status;
  action_server_feedback_cv_.notify_all();
}

TrackerNode::TrackingResult TrackerNode::getTrackingResult(const tf2::Stamped<tf2::Transform>& robot_to_odom,
                                                           const Eigen::Vector3d& prediction_offset,
                                                           const double odom_linear_vel,
                                                           const double odom_angular_vel) const
{
  if (path_header_.frame_id.size() == 0 || path_.size() == 0)
  {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }
  // Transform
  trajectory_tracker::Path2D lpath;
  double transform_delay = 0;
  tf2::Stamped<tf2::Transform> transform = robot_to_odom;
  try
  {
    tf2::Stamped<tf2::Transform> odom_to_path;
    tf2::fromMsg(tfbuf_.lookupTransform(frame_odom_, path_header_.frame_id, tf2::TimePointZero), odom_to_path);

    transform *= odom_to_path;
    transform_delay = tf2::durationToSec(tf2_ros::fromRclcpp(now()) - transform.stamp_);
    if (std::abs(transform_delay) > 0.1 && check_old_path_)
    {
      const double seconds1 = now().seconds();
      const double seconds2 = tf2::timeToSec(transform.stamp_);
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "Timestamp of the transform is too old %f %f", seconds1,
                            seconds2);
    }
    const float trans_yaw = tf2::getYaw(transform.getRotation());
    const Eigen::Transform<double, 2, Eigen::TransformTraits::AffineCompact> trans =
        Eigen::Translation2d(Eigen::Vector2d(transform.getOrigin().x(), transform.getOrigin().y())) *
        Eigen::Rotation2Dd(trans_yaw);

    for (size_t i = 0; i < path_.size(); i += path_step_)
      lpath.push_back(trajectory_tracker::Pose2D(trans * path_[i].pos_, trans_yaw + path_[i].yaw_, path_[i].velocity_));
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(get_logger(), "TF exception: %s", e.what());
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const Eigen::Vector2d origin_raw = prediction_offset.head<2>();
  const float yaw_raw = prediction_offset[2];

  const float yaw_predicted = w_lim_.get() * look_forward_ / 2;
  const Eigen::Vector2d origin =
      Eigen::Vector2d(std::cos(yaw_predicted), std::sin(yaw_predicted)) * v_lim_.get() * look_forward_;

  const double path_length = lpath.length();

  // Find nearest line strip
  const trajectory_tracker::Path2D::ConstIterator it_local_goal =
      lpath.findLocalGoal(lpath.cbegin() + path_step_done_, lpath.cend(), allow_backward_);

  const float max_search_range = (path_step_done_ > 0) ? tracking_search_range_ : initial_tracking_search_range_;
  const trajectory_tracker::Path2D::ConstIterator it_nearest =
      lpath.findNearest(lpath.cbegin() + path_step_done_, it_local_goal, origin, max_search_range, epsilon_);

  if (it_nearest == lpath.end())
  {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const int64_t i_nearest = std::distance(lpath.cbegin(), it_nearest);
  const int64_t i_nearest_prev = std::max(0l, i_nearest - 1);
  const int64_t i_local_goal = std::distance(lpath.cbegin(), it_local_goal);

  const Eigen::Vector2d pos_on_line =
      trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);
  const Eigen::Vector2d pos_on_line_raw =
      trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin_raw);

  const float linear_vel = std::isnan(lpath[i_nearest].velocity_) ? vel_[0] : lpath[i_nearest].velocity_;

  // Remained distance to the local goal
  float remain_local = lpath.remainedDistance(lpath.cbegin(), it_nearest, it_local_goal, pos_on_line);
  // Remained distance to the final goal
  float distance_remains = lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line);
  float distance_remains_raw = lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line_raw);
  if (path_length < no_pos_cntl_dist_)
    distance_remains = distance_remains_raw = remain_local = 0;

  // Signed distance error
  const float dist_err = trajectory_tracker::lineDistance(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);

  // Angular error
  const Eigen::Vector2d vec = lpath[i_nearest].pos_ - lpath[i_nearest_prev].pos_;
  float angle_remains = -atan2(vec[1], vec[0]);
  const float angle_pose = allow_backward_ ? lpath[i_nearest].yaw_ : -angle_remains;
  float sign_vel = 1.0;
  if (std::cos(-angle_remains) * std::cos(angle_pose) + std::sin(-angle_remains) * std::sin(angle_pose) < 0)
  {
    sign_vel = -1.0;
    angle_remains = angle_remains + M_PI;
  }
  angle_remains = trajectory_tracker::angleNormalized(angle_remains);

  // Curvature
  const float curv = lpath.getCurvature(it_nearest, it_local_goal, pos_on_line, curv_forward_);

  RCLCPP_DEBUG(get_logger(), "nearest: %ld, local goal: %ld, done: %ld, goal: %lu, remain: %0.3f, remain_local: %0.3f",
               i_nearest, i_local_goal, path_step_done_, lpath.size(), distance_remains, remain_local);

  bool arrive_local_goal(false);
  const bool in_place_turning_at_last = (vec[1] == 0.0 && vec[0] == 0.0);
  const bool in_place_turning_on_the_way = (std::abs(angle_remains) > stop_tolerance_ang_) && (v_lim_.get() == 0.0);

  TrackingResult result(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING);

  // Stop and rotate
  const bool large_angle_error = std::abs(rotate_ang_) < M_PI && std::cos(rotate_ang_) > std::cos(angle_remains);
  if (large_angle_error || std::abs(remain_local) < stop_tolerance_dist_ || path_length < min_track_path_ ||
      in_place_turning_at_last || in_place_turning_on_the_way)
  {
    if (large_angle_error)
    {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_DEBUG_THROTTLE(get_logger(), clock, 1000, "Stop and rotate due to large angular error: %0.3f",
                            angle_remains);
    }

    if (path_length < min_track_path_ || std::abs(remain_local) < stop_tolerance_dist_ || in_place_turning_at_last)
    {
      angle_remains = trajectory_tracker::angleNormalized(-(it_local_goal - 1)->yaw_);
      if (it_local_goal != lpath.end())
        arrive_local_goal = true;
    }
    if (path_length < stop_tolerance_dist_ || in_place_turning_at_last)
      distance_remains = distance_remains_raw = 0.0;

    result.turning_in_place = true;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
  }
  else
  {
    // Too far from given path
    float dist_from_path = dist_err;
    if (i_nearest == 0)
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    else if (i_nearest + 1 >= static_cast<int>(path_.size()))
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    if (std::abs(dist_from_path) > d_stop_)
    {
      result.distance_remains = distance_remains;
      result.distance_remains_raw = distance_remains_raw;
      result.angle_remains = angle_remains;
      result.angle_remains_raw = angle_remains + yaw_raw;
      result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH;
      return result;
    }

    // Path following control
    result.turning_in_place = false;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
    result.angle_remains_raw = angle_remains + yaw_raw;
    result.distance_from_target = trajectory_tracker::clip(dist_err, d_lim_);
    result.signed_local_distance = -remain_local * sign_vel;
    result.tracking_point_curv = curv;
    result.tracking_point_x = pos_on_line[0];
    result.tracking_point_y = pos_on_line[1];
  }

  if (std::abs(result.distance_remains) < goal_tolerance_dist_ &&
      std::abs(result.angle_remains) < goal_tolerance_ang_ &&
      std::abs(result.distance_remains_raw) < goal_tolerance_dist_ &&
      std::abs(result.angle_remains_raw) < goal_tolerance_ang_ &&
      (goal_tolerance_lin_vel_ == 0.0 || std::abs(odom_linear_vel) < goal_tolerance_lin_vel_) &&
      (goal_tolerance_ang_vel_ == 0.0 || std::abs(odom_angular_vel) < goal_tolerance_ang_vel_) &&
      it_local_goal == lpath.end())
  {
    result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL;
  }

  if (arrive_local_goal)
    result.path_step_done = i_local_goal;
  else
    result.path_step_done = std::max(path_step_done_, i_nearest - 1);

  return result;
}

void TrackerNode::publishZeroVelocity()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_vel_->publish(cmd_vel);
  v_lim_.clear();
  w_lim_.clear();
}

void TrackerNode::resetLatestStatus()
{
  latest_status_.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
  latest_status_.distance_remains = 0.0;
  latest_status_.angle_remains = 0.0;
  latest_status_.path_header = std_msgs::msg::Header();
  latest_status_.last_passed_index = 0;
  latest_status_.distance_to_path = 0.0;
}

static std::string to_string(const trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status)
{
  switch (status.status)
  {
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
      return "NO_PATH";
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH:
      return "FAR_FROM_PATH";
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING:
      return "FOLLOWING";
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL:
      return "GOAL";
    default:
      return "UNKNOWN";
  }
}

template <typename ActionClass>
void TrackerNode::computeControl(std::shared_ptr<nav2_util::SimpleActionServer<ActionClass>> action_server)
{
  if (action_server == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Action server is not initialized.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");
  {
    const std::lock_guard<std::mutex> lock(action_server_mutex_);
    resetLatestStatus();
    if (action_server->get_current_goal()->path.poses.size() <= 1)
    {
      RCLCPP_WARN(get_logger(), "Too short path is given. Stopping the robot.");
      publishZeroVelocity();
      action_server->succeeded_current();
      return;
    }
    cbPath(action_server->get_current_goal()->path);
  }

  const bool goal_reached = spinActionServer(action_server);
  const std::lock_guard<std::mutex> lock(action_server_mutex_);
  publishZeroVelocity();
  if (goal_reached)
  {
    // Reached the goal
    action_server->succeeded_current();
  }
  else
  {
    // Failed to reach the goal
    if (action_server->is_server_active())
    {
      action_server->terminate_all();
    }
  }
}

void TrackerNode::setFeedback(Action::Feedback& feedback)
{
  feedback.distance_to_goal = latest_status_.distance_remains;
  feedback.speed = v_lim_.get();
}

void TrackerNode::setFeedback(ActionWithVelocity::Feedback& feedback)
{
  feedback.speed = v_lim_.get();
  feedback.status = latest_status_;
}

void TrackerNode::publishRemainingPath()
{
  nav_msgs::msg::Path remaining_path;
  remaining_path.header = path_header_;
  if ((latest_status_.last_passed_index >= 0) && (latest_status_.last_passed_index < received_path_.poses.size()))
  {
    for (size_t i = path_step_done_ + 1; i < received_path_.poses.size(); ++i)
    {
      remaining_path.poses.push_back(received_path_.poses[i]);
    }
    // keep the last pose of the original path when remaining path is empty
    if (remaining_path.poses.empty() && (!received_path_.poses.empty()))
    {
      remaining_path.poses.push_back(received_path_.poses.back());
    }
    pub_remaining_path_->publish(remaining_path);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Invalid last_passed index value: %d, path size: %lu",
                latest_status_.last_passed_index, received_path_.poses.size());
  }
}

template <typename ActionClass>
bool TrackerNode::spinActionServer(const std::shared_ptr<nav2_util::SimpleActionServer<ActionClass>> action_server)
{
  try
  {
    int unable_to_follow_path_count = 0;
    int last_path_step_done = -1;
    rclcpp::Time previout_remaining_path_publish_time = now();
    while (rclcpp::ok())
    {
      std::unique_lock<std::mutex> lock(action_server_mutex_);
      if (!action_server->is_server_active())
      {
        RCLCPP_WARN(get_logger(), "Action server unavailable or inactive. Stopping.");
        return false;
      }
      if (action_server->is_cancel_requested())
      {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        return false;
      }
      if (action_server->is_preempt_requested())
      {
        RCLCPP_DEBUG(get_logger(), "Passing new path to controller.");
        auto goal = action_server->accept_pending_goal();
        cbPath(goal->path);
        unable_to_follow_path_count = 0;
        last_path_step_done = -1;
      }
      action_server_feedback_cv_.wait_for(lock, action_server_wait_duration_);
      if (!rclcpp::ok())
      {
        return false;
      }
      switch (latest_status_.status)
      {
        case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
        case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH:
          ++unable_to_follow_path_count;
          break;
        case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL:
          RCLCPP_INFO(get_logger(), "Reached the goal!");
          return true;
        default:
          unable_to_follow_path_count = 0;
          break;
      }
      if (unable_to_follow_path_count >= unable_to_follow_path_threshold_)
      {
        RCLCPP_WARN(get_logger(), "Unable to follow path. Stopping the robot.");
        return false;
      }
      RCLCPP_DEBUG(get_logger(),
                   "Feedback Status: %s, remaining distance: %0.3f, angle: %0.3f, index: %d, dist_to_path: %0.3f",
                   to_string(latest_status_).c_str(), latest_status_.distance_remains, latest_status_.angle_remains,
                   latest_status_.last_passed_index, latest_status_.distance_to_path);
      auto feedback = std::make_shared<typename ActionClass::Feedback>();
      setFeedback(*feedback);
      action_server->publish_feedback(feedback);
      if (last_path_step_done != path_step_done_)
      {
        publishRemainingPath();
        last_path_step_done = path_step_done_;
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception in action server: %s", e.what());
  }
  return false;
}

}  // namespace trajectory_tracker
