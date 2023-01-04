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
// Authors: Mehmet Dogru, Melike Tanrikulu

#ifndef LOCAL_POSE_PUBLISHER__LOCAL_POSE_PUBLISHER_HPP_
#define LOCAL_POSE_PUBLISHER__LOCAL_POSE_PUBLISHER_HPP_

#include "local_pose_publisher/lanelet2_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <boost/optional.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>

class LocalPosePublisher : public rclcpp::Node
{
public:
  explicit LocalPosePublisher(const rclcpp::NodeOptions & node_options);

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cp_pose_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_bin_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_goal_nav_sat_fix_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_cp_nav_sat_fix_;

  // Debug
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_raw_local_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_debug_pose_;

  std::string lanelet2_map_projector_type_;

private:
  void onMapBin(const autoware_auto_mapping_msgs::msg::HADMapBin ::ConstSharedPtr msg);
  void onGoalNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void onCheckpointNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void onDebugPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  boost::optional<geometry_msgs::msg::Pose> getClosestPose(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg);

  static geometry_msgs::msg::Point geographicCoordinatesToUTM(
    const double & latitude, const double & longitude, const double & altitude);

  static void refineAllCenterLines(
    lanelet::LaneletLayer & lanelet_layer, const double & center_line_density);

  static boost::optional<geometry_msgs::msg::Pose> getClosestCenterLinePoseFromLanelet(
    const lanelet::LaneletLayer & lanelet_layer, const geometry_msgs::msg::Point & p,
    const double & distance_threshold, const int & nearest_lanelet_count, const bool & debug_mode);

  void publishPoseStamped(
    const geometry_msgs::msg::Pose & pose,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_ptr);

  std::shared_ptr<lanelet::LaneletMap> map_;
  geometry_msgs::msg::Point utm_map_origin_;

  double center_line_resolution_;
  double distance_threshold_;
  int nearest_lanelet_count_;
  bool debug_mode_;

  bool map_ready_;
  bool goal_ready_;
  unsigned int cp_counter_;
};

#endif  // LOCAL_POSE_PUBLISHER__LOCAL_POSE_PUBLISHER_HPP_
