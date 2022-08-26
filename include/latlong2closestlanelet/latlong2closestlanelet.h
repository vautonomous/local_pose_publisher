// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef LATLONG2CLOSESTLANELET__LATLONG2CLOSESTLANELET_H_
#define LATLONG2CLOSESTLANELET__LATLONG2CLOSESTLANELET_H_

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>

class CartesianConv : public rclcpp::Node {
 public:
  CartesianConv();

  bool is_init = 0;
  geometry_msgs::msg::PoseWithCovarianceStamped utm_pose_map;


  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_to_pose_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix;

  double deg2rad = M_PI / 180;
 private:

 void navsatfix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
 geometry_msgs::msg::PoseWithCovarianceStamped gnss_baselink_pose;

};

#endif  // LATLONG2CLOSESTLANELET__LATLONG2CLOSESTLANELET_H_