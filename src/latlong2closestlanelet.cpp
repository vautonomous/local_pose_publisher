// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include "latlong2closestlanelet/latlong2closestlanelet.h"
#include <GeographicLib/UTMUPS.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace GeographicLib;

CartesianConvNode::CartesianConvNode(const rclcpp::NodeOptions &node_options)
    : Node("lat_lon_to_lanelet_closest_pose", node_options) {

  using std::placeholders::_1;

  // Parameters
  double origin_latitude =
      this->declare_parameter("origin_latitude", 40.81187906);
  double origin_longitude =
      this->declare_parameter("origin_longitude", 29.35810110);
  double origin_altitude = this->declare_parameter("origin_altitude", 48.35);

  // Initialize UTM map origin
  int zone;
  bool northp;
  UTMUPS::Forward(origin_latitude, origin_longitude, zone, northp,
                  utm_pose_map_->pose.position.x,
                  utm_pose_map_->pose.position.y);
  utm_pose_map_->pose.position.z = origin_altitude;

  // Publishers
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/output/goal_pose_on_lanelet", rclcpp::QoS{1}.transient_local());

  // Subscriptions
  sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "~/input/goal_gnss_coordinate", rclcpp::QoS{1}.transient_local(),
      std::bind(&CartesianConvNode::onNavSatFix, this, _1));
}

void CartesianConvNode::onNavSatFix(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {

  if (!utm_pose_map_) {
    return;
  }

  double latitude = msg->latitude;
  double longitude = msg->longitude;
  double altitude = msg->altitude;

  geometry_msgs::msg::PoseStamped utm_pose;
  geometry_msgs::msg::PoseStamped utm_pose_local;
  utm_pose_local.header.frame_id = "map";
  utm_pose_local.header.stamp = rclcpp::Clock().now();

  int zone;
  bool northp;
  UTMUPS::Forward(latitude, longitude, zone, northp, utm_pose.pose.position.x,
                  utm_pose.pose.position.y);

  utm_pose_local.pose.position.x =
      utm_pose.pose.position.x - utm_pose_map_->pose.position.x;
  utm_pose_local.pose.position.y =
      utm_pose.pose.position.y - utm_pose_map_->pose.position.y;
  utm_pose_local.pose.position.z = altitude - utm_pose_map_->pose.position.z;

  pub_pose_->publish(utm_pose_local);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<CartesianConvNode>(node_options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}