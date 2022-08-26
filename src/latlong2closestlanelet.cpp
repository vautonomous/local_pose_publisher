// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#include "latlong2closestlanelet/latlong2closestlanelet.h"
#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

using namespace GeographicLib;
CartesianConv::CartesianConv()
    : Node("SensorSubscriber") {
  std::cout.precision(20);

  map_to_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gnss_pose_local", 10);

  navsatfix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/applanix/lvx_client/gnss/fix",10,
          std::bind(&CartesianConv::navsatfix_cb, this, std::placeholders::_1));
}
void CartesianConv::navsatfix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

    if(is_init == 0) {
        double initial_map_lat = this->declare_parameter("latitude_map").template get<double>();
        double initial_map_lon = this->declare_parameter("longitude_map").template get<double>();
        double initial_map_alt = this->declare_parameter("altitude_map").template get<double>();

        int zone ;
        bool northp;
        UTMUPS::Forward(initial_map_lat, initial_map_lon, zone, northp, utm_pose_map.pose.pose.position.x, utm_pose_map.pose.pose.position.y);
        utm_pose_map.pose.pose.position.z = initial_map_alt;

        is_init = 1;
    }

    double Latitude  = msg->latitude;
    double Longitude = msg->longitude;
    double Altitude  = msg->altitude;

    geometry_msgs::msg::PoseWithCovarianceStamped utm_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped utm_pose_local;
    utm_pose_local.header.frame_id = "map";
    utm_pose_local.header.stamp = rclcpp::Clock().now();

    int zone ;
    bool northp;
    UTMUPS::Forward(Latitude, Longitude, zone, northp, utm_pose.pose.pose.position.x, utm_pose.pose.pose.position.y);

    utm_pose_local.pose.pose.position.x = utm_pose.pose.pose.position.x - utm_pose_map.pose.pose.position.x ;
    utm_pose_local.pose.pose.position.y = utm_pose.pose.pose.position.y - utm_pose_map.pose.pose.position.y;
    utm_pose_local.pose.pose.position.z = Altitude - utm_pose_map.pose.pose.position.z;


    map_to_pose_->publish(utm_pose_local);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianConv>());
  rclcpp::shutdown();
  return 0;
}

