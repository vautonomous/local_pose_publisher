// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef LATLONG2CLOSESTLANELET__LATLONG2CLOSESTLANELET_H_
#define LATLONG2CLOSESTLANELET__LATLONG2CLOSESTLANELET_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/optional.hpp>
#include <memory>

class CartesianConvNode : public rclcpp::Node {
public:
  explicit CartesianConvNode(const rclcpp::NodeOptions &node_options);

  geometry_msgs::msg::PoseStamped utm_pose_map_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_nav_sat_fix_;

private:
  void onNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
};

#endif // LATLONG2CLOSESTLANELET__LATLONG2CLOSESTLANELET_H_