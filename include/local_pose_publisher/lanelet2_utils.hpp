// Copyright 2015-2022 Autoware Foundation. All rights reserved.
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
// Authors: Kenji Miyake, Ryohsuke Mitsudome

#ifndef LOCAL_POSE_PUBLISHER__LANELET2_UTILS_HPP_
#define LOCAL_POSE_PUBLISHER__LANELET2_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include <memory>

namespace lanelet2_utils
{
lanelet::LineString3d generateFineCenterLine(
  const lanelet::ConstLanelet & lanelet_obj, const double & resolution);

std::vector<lanelet::BasicPoint3d> resamplePoints(
  const lanelet::ConstLineString3d & line_string, const int & num_segments);

std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string);

std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d & line_string);

std::pair<size_t, size_t> findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double & target_length);

boost::optional<size_t> findNearestIndex(
  const lanelet::ConstLineString3d & line, const geometry_msgs::msg::Point & point);

geometry_msgs::msg::Quaternion getOrientation(
  const lanelet::BasicPoint3d & point, const lanelet::BasicPoint3d & next_point);

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const geometry_msgs::msg::Quaternion & quaternion);

}  // namespace lanelet2_utils

#endif  // LOCAL_POSE_PUBLISHER__LANELET2_UTILS_HPP_
