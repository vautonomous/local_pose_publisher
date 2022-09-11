// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Mehmet Dogru

#ifndef LOCAL_POSE_PUBLISHER__LANELET2_UTILS_HPP_
#define LOCAL_POSE_PUBLISHER__LANELET2_UTILS_HPP_

#include "lanelet2_extension/utility/utilities.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <memory>

namespace lanelet2_utils
{
boost::optional<size_t> findNearestIndex(
  const lanelet::ConstLineString3d & line, const geometry_msgs::msg::Point & point);

geometry_msgs::msg::Quaternion getOrientation(
  const lanelet::BasicPoint3d & point, const lanelet::BasicPoint3d & next_point);

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const geometry_msgs::msg::Quaternion & quaternion);

}  // namespace lanelet2_utils

#endif  // LOCAL_POSE_PUBLISHER__LANELET2_UTILS_HPP_
