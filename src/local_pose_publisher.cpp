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

#include "local_pose_publisher/local_pose_publisher.hpp"

#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>

using GeographicLib::UTMUPS;

LocalPosePublisher::LocalPosePublisher(const rclcpp::NodeOptions & node_options)
: Node("local_pose_publisher", node_options), map_ready_(false), goal_ready_(false), cp_counter_(0)
{
  using std::placeholders::_1;

  // Parameters
  lanelet2_map_projector_type_ = declare_parameter("lanelet2_map_projector_type", "");

  if (lanelet2_map_projector_type_ == "UTM") {
    auto const origin_latitude = declare_parameter("latitude", 0.0);
    const double origin_longitude = declare_parameter("longitude", 0.0);
    const double origin_altitude =
      declare_parameter("local_pose_publisher.map_origin.altitude", 0.0);

    // Initialize UTM map origin
    utm_map_origin_ =
      geographicCoordinatesToUTM(origin_latitude, origin_longitude, origin_altitude);
  }

  center_line_resolution_ = declare_parameter("local_pose_publisher.center_line_resolution", 1.0);
  distance_threshold_ = declare_parameter("local_pose_publisher.distance_threshold", 30.0);
  debug_mode_ = declare_parameter("local_pose_publisher.debug_mode", false);
  nearest_lanelet_count_ =
    static_cast<int>(declare_parameter("local_pose_publisher.advanced.nearest_lanelet_count", 10));

  // Publishers
  pub_goal_pose_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/goal_pose_on_lanelet", 1);

  pub_cp_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/output/checkpoint_pose_on_lanelet", 1);

  // Subscriptions
  sub_map_bin_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&LocalPosePublisher::onMapBin, this, _1));

  sub_goal_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "~/input/goal_gnss_coordinate", 10, std::bind(&LocalPosePublisher::onGoalNavSatFix, this, _1));

  sub_cp_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "~/input/checkpoint_gnss_coordinate", 10,
    std::bind(&LocalPosePublisher::onCheckpointNavSatFix, this, _1));

  if (debug_mode_) {
    pub_raw_local_point_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("~/output/debug/raw_local_point", 1);

    sub_debug_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/input/debug/pose", 10, std::bind(&LocalPosePublisher::onDebugPose, this, _1));
  }
}

void LocalPosePublisher::onMapBin(
  const autoware_auto_mapping_msgs::msg::HADMapBin ::ConstSharedPtr msg)
{
  map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map_);
  refineAllCenterLines(map_->laneletLayer, center_line_resolution_);
  map_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Lanelet2 map is loaded successfully.");
}

void LocalPosePublisher::onGoalNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (!map_ready_) {
    return;
  }

  auto closest_pose = getClosestPose(msg);
  if (closest_pose) {
    // Publish the closest goal pose
    publishPoseStamped(*closest_pose, pub_goal_pose_);

    cp_counter_ = 0;
    goal_ready_ = true;

    RCLCPP_INFO(this->get_logger(), "Goal pose is published.");
  }
}

void LocalPosePublisher::onCheckpointNavSatFix(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (!goal_ready_) {
    RCLCPP_WARN(this->get_logger(), "First give a goal point! Checkpoint is discarded.");
    return;
  }

  auto closest_pose = getClosestPose(msg);
  if (closest_pose) {
    // Publish the closest checkpoint pose
    publishPoseStamped(*closest_pose, pub_cp_pose_);
    RCLCPP_INFO(this->get_logger(), "Checkpoint pose is published (%d)", cp_counter_);
  }
}

void LocalPosePublisher::onDebugPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!map_ready_) {
    return;
  }

  auto closest_pose = getClosestCenterLinePoseFromLanelet(
    map_->laneletLayer, msg->pose.position, distance_threshold_, nearest_lanelet_count_,
    debug_mode_);

  if (closest_pose) {
    shiftPoseToVehicleLateralCenter(*closest_pose);

    // Publish the closest pose
    publishPoseStamped(*closest_pose, pub_goal_pose_);
    RCLCPP_INFO(this->get_logger(), "Goal pose is published.");
  }
}

boost::optional<geometry_msgs::msg::Pose> LocalPosePublisher::getClosestPose(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg)
{
  double latitude = msg->latitude;
  double longitude = msg->longitude;
  double altitude = msg->altitude;

  auto utm_point = geographicCoordinatesToUTM(latitude, longitude, altitude);

  // Calculate local point
  geometry_msgs::msg::Point utm_local_point;
  utm_local_point.x = utm_point.x - utm_map_origin_.x;
  utm_local_point.y = utm_point.y - utm_map_origin_.y;
  utm_local_point.z = altitude - utm_map_origin_.z;

  // [debug] Publish raw local point
  if (debug_mode_) {
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.frame_id = "map";
    point_stamped.header.stamp = this->now();
    point_stamped.point = utm_local_point;
    pub_raw_local_point_->publish(point_stamped);
  }

  // Find closest center line pose from lanelet2 map
  auto closest_pose = getClosestCenterLinePoseFromLanelet(
    map_->laneletLayer, utm_local_point, distance_threshold_, nearest_lanelet_count_, debug_mode_);

  if (closest_pose) {
    shiftPoseToVehicleLateralCenter(*closest_pose);
    return closest_pose;
  }

  return {};
}

geometry_msgs::msg::Point LocalPosePublisher::geographicCoordinatesToUTM(
  const double & latitude, const double & longitude, const double & altitude)
{
  geometry_msgs::msg::Point utm_point;
  int zone;
  bool northp;
  UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.x, utm_point.y);
  utm_point.z = altitude;

  return utm_point;
}

void LocalPosePublisher::refineAllCenterLines(
  lanelet::LaneletLayer & lanelet_layer, const double & center_line_density)
{
  for (auto & llt : lanelet_layer) {
    const auto refined_center_line =
      lanelet::utils::generateFineCenterline(llt, center_line_density);
    llt.setCenterline(refined_center_line);
  }
}

boost::optional<geometry_msgs::msg::Pose> LocalPosePublisher::getClosestCenterLinePoseFromLanelet(
  const lanelet::LaneletLayer & lanelet_layer, const geometry_msgs::msg::Point & p,
  const double & distance_threshold, const int & nearest_lanelet_count, const bool & debug_mode)
{
  lanelet::BasicPoint2d lanelet_point(p.x, p.y);
  auto const close_lanelets =
    lanelet::geometry::findNearest(lanelet_layer, lanelet_point, nearest_lanelet_count);

  if (debug_mode) {
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Closest lanelets:" << std::endl;
  }
  for (auto const & llt_pair : close_lanelets) {
    auto const distance_to_lanelet = llt_pair.first;
    auto lanelet = llt_pair.second;

    std::string subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, "none");

    if (debug_mode) {
      std::cout << "-----------------" << std::endl;
      std::cout << "lanelet.id(): " << lanelet.id() << std::endl;
      std::cout << "subtype: " << subtype << std::endl;
      std::cout << "distance_to_lanelet: " << distance_to_lanelet << std::endl;
      std::cout << "-----------------" << std::endl;
    }

    if (subtype != std::string("road")) {
      continue;
    }

    if (distance_to_lanelet > distance_threshold) {
      std::cout << "Coordinate is too far from the closest lanelet!";
      if (debug_mode) {
        std::cout << " [distance_to_lanelet(" << distance_to_lanelet << ") > distance_threshold("
                  << distance_threshold << ")]";
      }
      std::cout << "" << std::endl;
      return {};
    }

    auto nearest_idx = lanelet2_utils::findNearestIndex(lanelet.centerline(), p);

    if (!nearest_idx) {
      return {};
    }

    if (lanelet.centerline().size() - *nearest_idx == 1) {
      nearest_idx = std::max(static_cast<int>(*nearest_idx) - 1, 0);
    }

    auto const nearest_point = lanelet.centerline()[*nearest_idx];
    auto const nearest_point_next = lanelet.centerline()[*nearest_idx + 1];

    auto const pose_orientation = lanelet2_utils::getOrientation(nearest_point, nearest_point_next);

    auto closest_pose = lanelet2_utils::convertBasicPoint3dToPose(nearest_point, pose_orientation);
    return closest_pose;
  }
  if (debug_mode) {
    std::cout << "Couldn't find the closest center line from lanelet! (Consider "
                 "increasing #count for lanelet::geometry::findNearest)"
              << std::endl;
  }
  return {};
}

void LocalPosePublisher::publishPoseStamped(
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_ptr)
{
  geometry_msgs::msg::PoseStamped closest_pose_stamped;
  closest_pose_stamped.header.frame_id = "map";
  closest_pose_stamped.header.stamp = this->now();
  closest_pose_stamped.pose = pose;

  pub_ptr->publish(closest_pose_stamped);
}

void LocalPosePublisher::shiftPoseToVehicleLateralCenter(geometry_msgs::msg::Pose & pose)
{
  const auto pose_copy = pose;

  tf2::Quaternion q;
  tf2::convert(pose_copy.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  const auto shift_length = (vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;

  const auto delta_x = -1 * shift_length * sin(yaw);
  const auto delta_y = shift_length * cos(yaw);

  pose.position.x += delta_x;
  pose.position.y += delta_y;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<LocalPosePublisher>(node_options);
  if (node->lanelet2_map_projector_type_ != "UTM") {
    RCLCPP_ERROR(node->get_logger(), "Map origin coordinates should be in UTM coordinate frame!");
    rclcpp::shutdown();
    return 0;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}