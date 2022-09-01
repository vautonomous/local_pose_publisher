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

#include "latlong2closestlanelet/lanelet2_utils.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace lanelet2_utils {

lanelet::LineString3d
generateFineCenterLine(const lanelet::ConstLanelet &lanelet_obj,
                       const double &resolution) {
  // Get length of longer border
  const double left_length =
      static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
      static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance =
      (left_length > right_length) ? left_length : right_length;
  const int num_segments =
      std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points =
      resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points =
      resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create center line
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (int i = 0; i < num_segments + 1; i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point =
        (right_points.at(i) + left_points.at(i)) / 2;
    const lanelet::Point3d center_point(
        lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
        center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

std::vector<lanelet::BasicPoint3d>
resamplePoints(const lanelet::ConstLineString3d &line_string,
               const int &num_segments) {
  // Calculate length
  const auto line_length =
      static_cast<double>(lanelet::geometry::length(line_string));

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);
  if (accumulated_lengths.size() < 2)
    return {};

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const auto target_length =
        (static_cast<double>(i) / num_segments) * line_length;
    const auto index_pair =
        findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point =
        back_point +
        (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.emplace_back(target_point);
  }

  return resampled_points;
}

std::vector<double>
calculateAccumulatedLengths(const lanelet::ConstLineString3d &line_string) {
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(std::begin(segment_distances), std::end(segment_distances),
                   std::back_inserter(accumulated_lengths));

  return accumulated_lengths;
}

std::vector<double>
calculateSegmentDistances(const lanelet::ConstLineString3d &line_string) {
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);

  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance =
        lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }

  return segment_distances;
}

std::pair<size_t, size_t>
findNearestIndexPair(const std::vector<double> &accumulated_lengths,
                     const double &target_length) {
  // List size
  const auto N = accumulated_lengths.size();

  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }

  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (std::size_t i = 1; i < N; ++i) {
    if (accumulated_lengths.at(i - 1) <= target_length &&
        target_length <= accumulated_lengths.at(i)) {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  throw std::runtime_error("No nearest point found.");
}

boost::optional<size_t>
findNearestIndex(const lanelet::ConstLineString3d &line,
                 const geometry_msgs::msg::Point &point) {

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

geometry_msgs::msg::Quaternion
getOrientation(const lanelet::BasicPoint3d &point,
               const lanelet::BasicPoint3d &next_point) {

  auto angle =
      std::atan2(next_point.y() - point.y(), next_point.x() - point.x());

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0, 0, angle);

  geometry_msgs::msg::Quaternion q;
  q.w = tf2_q.getW();
  q.x = tf2_q.getX();
  q.y = tf2_q.getY();
  q.z = tf2_q.getZ();

  return q;
}

geometry_msgs::msg::Pose
convertBasicPoint3dToPose(const lanelet::BasicPoint3d &point,
                          const geometry_msgs::msg::Quaternion &quaternion) {

  geometry_msgs::msg::Pose pose;

  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();

  pose.orientation = quaternion;

  return pose;
}

} // namespace lanelet2_utils