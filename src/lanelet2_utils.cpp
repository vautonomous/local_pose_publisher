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

#include "local_pose_publisher/lanelet2_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace lanelet2_utils
{
boost::optional<size_t> findNearestIndex(
  const lanelet::ConstLineString3d & line, const geometry_msgs::msg::Point & point)
{
  if (line.empty()) {
    return {};
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < line.size(); ++i) {
    geometry_msgs::msg::Point p1;
    p1.x = line[i].basicPoint().x();
    p1.y = line[i].basicPoint().y();

    const auto dx = p1.x - point.x;
    const auto dy = p1.y - point.y;
    const auto squared_distance = dx * dx + dy * dy;

    if (squared_distance < min_dist) {
      min_dist = squared_distance;
      min_idx = i;
    }
  }
  return min_idx;
}

geometry_msgs::msg::Quaternion getOrientation(
  const lanelet::BasicPoint3d & point, const lanelet::BasicPoint3d & next_point)
{
  auto angle = std::atan2(next_point.y() - point.y(), next_point.x() - point.x());

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0, 0, angle);

  geometry_msgs::msg::Quaternion q;
  q.w = tf2_q.getW();
  q.x = tf2_q.getX();
  q.y = tf2_q.getY();
  q.z = tf2_q.getZ();

  return q;
}

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const geometry_msgs::msg::Quaternion & quaternion)
{
  geometry_msgs::msg::Pose pose;

  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();

  pose.orientation = quaternion;

  return pose;
}

}  // namespace lanelet2_utils