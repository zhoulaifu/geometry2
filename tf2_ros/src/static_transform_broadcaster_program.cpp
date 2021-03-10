/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster_node.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

std::unordered_map<std::string, std::string> _make_arg_map(std::vector<std::string> && args)
{
  std::unordered_map<std::string, std::string> ret;
  /* collect from [exe] --option1 value --option2 value ... --optionN value */
  for (size_t x = 1; x < args.size(); x += 2) {
    ret.emplace(std::move(args[x]), std::move(args[x + 1]));
  }
  return ret;
}

void _print_usage()
{
  /* TODO(allenh1): update the usage */
  printf("A command line utility for manually sending a transform.\n");
  printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id \n");
  printf("OR \n");
  printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id \n");
  RCUTILS_LOG_ERROR(
    "static_transform_publisher exited due to not having the right number of arguments");
}

tf2::Quaternion _get_rotation(const std::unordered_map<std::string, std::string> & args)
{
  tf2::Quaternion quat;
  auto iter = args.find("--qx");
  if (iter != args.end()) {
    quat.setX(std::stod(iter->second));
  }
  iter = args.find("--qy");
  if (iter != args.end()) {
    quat.setY(std::stod(iter->second));
  }
  iter = args.find("--qz");
  if (iter != args.end()) {
    quat.setZ(std::stod(iter->second));
  }
  iter = args.find("--qw");
  if (iter != args.end()) {
    quat.setW(std::stod(iter->second));
  }
  /* TODO(allenh1): Parse RPY */
  return quat;
}

tf2::Vector3 _get_translation(const std::unordered_map<std::string, std::string> & args)
{
  tf2::Vector3 trans;
  auto iter = args.find("--x");
  if (iter != args.end()) {
    trans.setX(std::stod(iter->second));
  }
  iter = args.find("--y");
  if (iter != args.end()) {
    trans.setY(std::stod(iter->second));
  }
  iter = args.find("--z");
  if (iter != args.end()) {
    trans.setZ(std::stod(iter->second));
  }
  return trans;
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::unordered_map<std::string, std::string> arg_map = _make_arg_map(std::move(args));
  rclcpp::NodeOptions options;
  std::shared_ptr<tf2_ros::StaticTransformBroadcasterNode> node;
  tf2::Quaternion rotation = _get_rotation(arg_map);
  tf2::Vector3 translation = _get_translation(arg_map);
  std::string frame_id, child_id;
  auto iter = arg_map.find("--frame-id");
  if (iter == arg_map.end()) {
    _print_usage();
    return 1;
  }
  frame_id = iter->second;
  iter = arg_map.find("--child-frame-id");
  if (iter == arg_map.end()) {
    _print_usage();
    return 1;
  }
  child_id = iter->second;

  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"translation.x", translation.x()},
    {"translation.y", translation.y()},
    {"translation.z", translation.z()},
    {"rotation.x", rotation.x()},
    {"rotation.y", rotation.y()},
    {"rotation.z", rotation.z()},
    {"rotation.w", rotation.w()},
    {"frame_id", frame_id},
    {"child_frame_id", child_id},
  });

  node = std::make_shared<tf2_ros::StaticTransformBroadcasterNode>(options);

  RCLCPP_INFO(
    node->get_logger(), "Spinning until killed publishing transform from '%s' to '%s'",
    frame_id.c_str(), child_id.c_str());
  rclcpp::spin(node);
  return 0;
}
