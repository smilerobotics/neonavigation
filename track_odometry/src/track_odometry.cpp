/*
 * Copyright (c) 2014-2019, the neonavigation authors
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

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "neonavigation_common/neonavigation_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "track_odometry/kalman_filter1.hpp"

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion& a)
{
  return Eigen::Quaterniond(a.w, a.x, a.y, a.z);
}
geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& a)
{
  geometry_msgs::msg::Point b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}
geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d& a)
{
  geometry_msgs::msg::Vector3 b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}

class TrackOdometryNode : public neonavigation_common::NeonavigationNode
{
private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                                     sensor_msgs::msg::Imu>;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> sub_imu_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_reset_z_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Odometry odom_prev_;
  nav_msgs::msg::Odometry odomraw_prev_;

  std::string base_link_id_;
  std::string base_link_id_overwrite_;
  std::string odom_id_;

  sensor_msgs::msg::Imu imu_;
  double gyro_zero_[3];
  double z_filter_timeconst_;
  double tf_tolerance_;

  bool debug_;
  bool use_kf_;
  bool negative_slip_;
  double sigma_predict_;
  double sigma_odom_;
  double predict_filter_tc_;
  double dist_;
  bool without_odom_;

  track_odometry::KalmanFilter1 slip_;

  bool has_imu_;
  bool has_odom_;
  bool publish_tf_;

  void cbResetZ(const std_msgs::msg::Float32::ConstSharedPtr& msg)
  {
    odom_prev_.pose.pose.position.z = msg->data;
  }
  void cbOdomImu(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
                 const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Synchronized timestamp: odom %0.3f, imu %0.3f",
                 rclcpp::Time(odom_msg->header.stamp).seconds(),
                 rclcpp::Time(imu_msg->header.stamp).seconds());
    cbImu(imu_msg);
    cbOdom(odom_msg);
  }
  void cbImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    if (base_link_id_.size() == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "base_link id is not specified.");
      return;
    }

    imu_.header = msg->header;
    try
    {
      const geometry_msgs::msg::TransformStamped trans =
          tf_buffer_->lookupTransform(base_link_id_, msg->header.frame_id, msg->header.stamp,
                                      rclcpp::Duration::from_seconds(0.1));

      geometry_msgs::msg::Vector3Stamped vin, vout;
      vin.header = imu_.header;
      vin.header.stamp = rclcpp::Time(0);
      vin.vector = msg->linear_acceleration;
      tf2::doTransform(vin, vout, trans);
      imu_.linear_acceleration = vout.vector;

      vin.header = imu_.header;
      vin.header.stamp = rclcpp::Time(0);
      vin.vector = msg->angular_velocity;
      tf2::doTransform(vin, vout, trans);
      imu_.angular_velocity = vout.vector;

      tf2::Stamped<tf2::Quaternion> qin, qout;
      geometry_msgs::msg::QuaternionStamped qmin, qmout;
      qmin.header = imu_.header;
      qmin.quaternion = msg->orientation;
      tf2::fromMsg(qmin, qin);

      auto axis = qin.getAxis();
      auto angle = qin.getAngle();
      geometry_msgs::msg::Vector3Stamped axis2;
      geometry_msgs::msg::Vector3Stamped axis1;
      axis1.vector = tf2::toMsg(axis);
      axis1.header.stamp = rclcpp::Time(0);
      axis1.header.frame_id = qin.frame_id_;
      tf2::doTransform(axis1, axis2, trans);

      tf2::fromMsg(axis2.vector, axis);
      qout.setData(tf2::Quaternion(axis, angle));
      qout.stamp_ = qin.stamp_;
      qout.frame_id_ = base_link_id_;

      qmout = tf2::toMsg(qout);
      imu_.orientation = qmout.quaternion;
      // RCLCPP_INFO(this->get_logger(), "%0.3f %s -> %0.3f %s", tf2::getYaw(qmin.quaternion),
      //             qmin.header.frame_id.c_str(), tf2::getYaw(qmout.quaternion),
      //             qmout.header.frame_id.c_str());

      has_imu_ = true;
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      has_imu_ = false;
      return;
    }
  }
  void cbOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    nav_msgs::msg::Odometry odom = *msg;
    if (has_odom_)
    {
      const double dt =
          (rclcpp::Time(odom.header.stamp) - rclcpp::Time(odomraw_prev_.header.stamp)).seconds();
      if (base_link_id_overwrite_.size() == 0)
      {
        base_link_id_ = odom.child_frame_id;
      }

      if (!has_imu_)
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "IMU data not received");
        return;
      }

      double slip_ratio = 1.0;
      odom.header.stamp =
          rclcpp::Time(odom.header.stamp) + rclcpp::Duration::from_seconds(tf_tolerance_);
      odom.twist.twist.angular = imu_.angular_velocity;
      odom.pose.pose.orientation = imu_.orientation;

      double w_imu = imu_.angular_velocity.z;
      const double w_odom = msg->twist.twist.angular.z;

      if (w_imu * w_odom < 0 && !negative_slip_)
        w_imu = w_odom;

      slip_.predict(-slip_.x_ * dt * predict_filter_tc_, dt * sigma_predict_);
      if (std::abs(w_odom) > sigma_odom_ * 3)
      {
        // non-kf mode: calculate slip_ratio if angular vel < 3*sigma
        slip_ratio = w_imu / w_odom;
      }

      const double slip_ratio_per_angvel = (w_odom - w_imu) / (w_odom * std::abs(w_odom));
      double slip_ratio_per_angvel_sigma =
          sigma_odom_ * std::abs(2.0 * w_odom * sigma_odom_ /
                                 std::pow(w_odom * w_odom - sigma_odom_ * sigma_odom_, 2));
      if (std::abs(w_odom) < sigma_odom_)
        slip_ratio_per_angvel_sigma = std::numeric_limits<double>::infinity();

      slip_.measure(slip_ratio_per_angvel, slip_ratio_per_angvel_sigma);
      // printf("%0.5f %0.5f %0.5f   %0.5f %0.5f  %0.5f\n",
      //   slip_ratio_per_angvel, slip_ratio_sigma, slip_ratio_per_angvel_sigma,
      //   slip_.x_, slip_.sigma_, msg->twist.twist.angular.z);

      if (debug_)
      {
        printf("%0.3f %0.3f  %0.3f  %0.3f %0.3f  %0.3f  %0.3f\n", imu_.angular_velocity.z,
               msg->twist.twist.angular.z, slip_ratio, slip_.x_, slip_.sigma_,
               odom.twist.twist.linear.x, dist_);
      }
      dist_ += odom.twist.twist.linear.x * dt;

      const Eigen::Vector3d diff =
          toEigen(msg->pose.pose.position) - toEigen(odomraw_prev_.pose.pose.position);
      Eigen::Vector3d v = toEigen(odom.pose.pose.orientation) *
                          toEigen(msg->pose.pose.orientation).inverse() * diff;
      if (use_kf_)
        v *= 1.0 - slip_.x_;
      else
        v *= slip_ratio;

      odom.pose.pose.position = toPoint(toEigen(odom_prev_.pose.pose.position) + v);
      if (z_filter_timeconst_ > 0)
        odom.pose.pose.position.z *= 1.0 - (dt / z_filter_timeconst_);

      odom.child_frame_id = base_link_id_;
      pub_odom_->publish(odom);

      geometry_msgs::msg::TransformStamped odom_trans;

      odom_trans.header = odom.header;
      odom_trans.child_frame_id = base_link_id_;
      odom_trans.transform.translation = toVector3(toEigen(odom.pose.pose.position));
      odom_trans.transform.rotation = odom.pose.pose.orientation;
      if (publish_tf_)
        tf_broadcaster_->sendTransform(odom_trans);
    }
    odomraw_prev_ = *msg;
    odom_prev_ = odom;
    has_odom_ = true;
  }

public:
  TrackOdometryNode(const std::string& name, const rclcpp::NodeOptions& options)
    : neonavigation_common::NeonavigationNode(name, options)
    , tf_buffer_(nullptr)
    , tf_listener_(nullptr)
    , tf_broadcaster_(nullptr)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    bool without_odom_ = this->declare_parameter("without_odom", false);
    if (without_odom_)
    {
      sub_imu_raw_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu/data", 64, std::bind(&TrackOdometryNode::cbImu, this, std::placeholders::_1));
      declare_dynamic_parameter("base_link_id", &base_link_id_, std::string("base_link"));
      declare_dynamic_parameter("odom_id", &odom_id_, std::string("odom"));
    }
    else
    {
      sub_odom_ =
          std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "odom_raw");
      sub_imu_ =
          std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, "imu/data");

      const int sync_window = this->declare_parameter("sync_window", 50);
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(sync_window),
                                                                          *sub_odom_, *sub_imu_);
      sync_->setAgePenalty(0.50);
      sync_->registerCallback(std::bind(&TrackOdometryNode::cbOdomImu, this, std::placeholders::_1,
                                        std::placeholders::_2));

      declare_dynamic_parameter("base_link_id", &base_link_id_overwrite_, std::string(""));
    }

    sub_reset_z_ = this->create_subscription<std_msgs::msg::Float32>(
        "reset_odometry_z", 1,
        std::bind(&TrackOdometryNode::cbResetZ, this, std::placeholders::_1));
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 8);

    declare_dynamic_parameter("z_filter_timeconst", &z_filter_timeconst_, -1.0);
    declare_dynamic_parameter("tf_tolerance", &tf_tolerance_, 0.01);
    declare_dynamic_parameter("use_kf", &use_kf_, true);
    declare_dynamic_parameter("enable_negative_slip", &negative_slip_, false);
    declare_dynamic_parameter("debug", &debug_, false);
    declare_dynamic_parameter("publish_tf", &publish_tf_, true);

    if (base_link_id_overwrite_.size() > 0)
    {
      base_link_id_ = base_link_id_overwrite_;
    }

    // sigma_odom_ [rad/s]: standard deviation of odometry angular vel on straight running
    declare_dynamic_parameter("sigma_odom", &sigma_odom_, 0.005);
    // sigma_predict_ [sigma/second]: prediction sigma of kalman filter
    declare_dynamic_parameter("sigma_predict", &sigma_predict_, 0.5);
    // predict_filter_tc_ [sec.]: LPF time-constant to forget estimated slip_ ratio
    declare_dynamic_parameter("predict_filter_tc", &predict_filter_tc_, 1.0);

    has_imu_ = false;
    has_odom_ = false;

    dist_ = 0;
    slip_.set(0.0, 0.1);

    if (without_odom_)
    {
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&TrackOdometryNode::cbTimer, this));
    }
  }
  void cbTimer()
  {
    nav_msgs::msg::Odometry::SharedPtr odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = this->get_clock()->now();
    odom->header.frame_id = odom_id_;
    odom->child_frame_id = base_link_id_;
    odom->pose.pose.orientation.w = 1.0;
    cbOdom(odom);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<TrackOdometryNode>("track_odometry", rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
