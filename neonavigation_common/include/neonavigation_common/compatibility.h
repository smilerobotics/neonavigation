/*
 * Copyright (c) 2018, the neonavigation authors
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

#ifndef NEONAVIGATION_COMMON_COMPATIBILITY_H
#define NEONAVIGATION_COMMON_COMPATIBILITY_H

#include "rclcpp/rclcpp.hpp"

#include <string>

namespace neonavigation_common
{
namespace compat
{
#define STATIC_ASSERT(EXPR) static_assert(EXPR, #EXPR)

// Update cycle
// 1. Increment current_level and indicate topic changes using mcl_3dl_compat::subscribe/advertise.
//    Set default_level to supported_level.
// 2. Set default_level to current_level.
// 3. Increment supported_level and remove old topic names.
#ifndef UNDEF_COMPATIBILITY_LEVEL
const int current_level = 1;
const int supported_level = 0;
const int default_level = supported_level;

STATIC_ASSERT(supported_level <= current_level && current_level <= supported_level + 1);
STATIC_ASSERT(supported_level <= default_level && default_level <= current_level);
#endif

int getCompat()
{
  int compat(default_level);
  rclcpp::Node("/").param("neonavigation_compatible", compat, compat);

  return compat;
}
void checkCompatMode()
{
  if (getCompat() < supported_level)
  {
    const std::string message =
        "======= [Obsolated] your configuration for " + ros::this_node::getName() + " is outdated =======";
    ROS_FATAL("%s", message.c_str());
    ros::shutdown();
    throw std::runtime_error(message);
  }
  else if (getCompat() > current_level)
  {
    const std::string message =
        "======= [Unsupported] your configuration for " + ros::this_node::getName() + " is futuredated =======";
    ROS_FATAL("%s", message.c_str());
    ros::shutdown();
    throw std::runtime_error(message);
  }
  else if (getCompat() != current_level)
  {
    ROS_ERROR_ONCE(
        "======= [Deprecated] %s is run in compatible mode =======\n"
        "=========================================================\n"
        "Set _compatible:=%d to switch to new topic namespaces.\n"
        "Compatible mode will be obsolated in the future update.\n"
        "=========================================================",
        ros::this_node::getName().c_str(), current_level);
  }
}
std::string getSimplifiedNamespace(rclcpp::Node& nh)
{
  if (nh.getUnresolvedNamespace() == ros::this_node::getName())
    return std::string("~/");
  if (nh.getUnresolvedNamespace() == std::string())
    return std::string();
  return nh.getNamespace() + "/";
}
template <class M>
ros::Subscriber subscribe(
    rclcpp::Node& nh_new,
    const std::string& topic_new,
    rclcpp::Node& nh_old,
    const std::string& topic_old,
    uint32_t queue_size,
    void (*fp)(M),
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  if (getCompat() != current_level)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.subscribe(topic_old, queue_size, fp, transport_hints);
  }
  else
  {
    return nh_new.subscribe(topic_new, queue_size, fp, transport_hints);
  }
}
template <class M, class T>
ros::Subscriber subscribe(
    rclcpp::Node& nh_new,
    const std::string& topic_new,
    rclcpp::Node& nh_old,
    const std::string& topic_old,
    uint32_t queue_size,
    void (T::*fp)(M) const,
    T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  if (getCompat() != current_level)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.subscribe(topic_old, queue_size, fp, obj, transport_hints);
  }
  else
  {
    return nh_new.subscribe(topic_new, queue_size, fp, obj, transport_hints);
  }
}
template <class M, class T>
ros::Subscriber subscribe(
    rclcpp::Node& nh_new,
    const std::string& topic_new,
    rclcpp::Node& nh_old,
    const std::string& topic_old,
    uint32_t queue_size,
    void (T::*fp)(M),
    T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  if (getCompat() != current_level)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.subscribe(topic_old, queue_size, fp, obj, transport_hints);
  }
  else
  {
    return nh_new.subscribe(topic_new, queue_size, fp, obj, transport_hints);
  }
}
template <class M>
ros::Subscriber subscribe(
    rclcpp::Node& nh_new,
    const std::string& topic_new,
    rclcpp::Node& nh_old,
    const std::string& topic_old,
    uint32_t queue_size,
    const std::function<void(const std::shared_ptr<M const>&)>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
    const ros::TransportHints& transport_hints = ros::TransportHints())
{
  if (getCompat() != current_level)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.subscribe(topic_old, queue_size, callback, tracked_object, transport_hints);
  }
  else
  {
    return nh_new.subscribe(topic_new, queue_size, callback, tracked_object, transport_hints);
  }
}
template <class M>
ros::Publisher advertise(
    rclcpp::Node& nh_new,
    const std::string& topic_new,
    rclcpp::Node& nh_old,
    const std::string& topic_old,
    uint32_t queue_size,
    bool latch = false)
{
  if (getCompat() != current_level)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use %s (%s%s) topic instead of %s (%s%s)",
        nh_new.resolveName(topic_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), topic_new.c_str(),
        nh_old.resolveName(topic_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), topic_old.c_str());
    return nh_old.advertise<M>(topic_old, queue_size, latch);
  }
  else
  {
    return nh_new.advertise<M>(topic_new, queue_size, latch);
  }
}
template <class T, class MReq, class MRes>
ros::ServiceServer advertiseService(
    rclcpp::Node& nh_new,
    const std::string& service_new,
    rclcpp::Node& nh_old,
    const std::string& service_old,
    bool (T::*srv_func)(MReq&, MRes&),
    T* obj)
{
  if (getCompat() != current_level)
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use %s (%s%s) service instead of %s (%s%s)",
        nh_new.resolveName(service_new, false).c_str(),
        getSimplifiedNamespace(nh_new).c_str(), service_new.c_str(),
        nh_old.resolveName(service_old, false).c_str(),
        getSimplifiedNamespace(nh_old).c_str(), service_old.c_str());
    return nh_old.advertiseService(service_old, srv_func, obj);
  }
  else
  {
    return nh_new.advertiseService(service_new, srv_func, obj);
  }
}

template <typename T>
void deprecatedParam(
    const rclcpp::Node& nh,
    const std::string& key,
    T& param,
    const T& default_value)
{
  if (nh.hasParam(key))
  {
    RCLCPP_ERROR(rclcpp::get_logger("NeonavigationCommon"), 
        "Use of the parameter %s is deprecated. Don't use this.",
        nh.resolveName(key, false).c_str());
  }
  nh.param(key, param, default_value);
}
}  // namespace compat
}  // namespace neonavigation_common

#endif  // NEONAVIGATION_COMMON_COMPATIBILITY_H
