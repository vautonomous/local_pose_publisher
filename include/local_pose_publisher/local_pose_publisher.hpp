// Copyright (c) 2022 Leo Drive Teknoloji A.Ş.

#ifndef LOCAL_POSE_PUBLISHER__LOCAL_POSE_PUBLISHER_HPP_
#define LOCAL_POSE_PUBLISHER__LOCAL_POSE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include <boost/optional.hpp>
#include <memory>

class LocalPosePublisher : public rclcpp::Node {
public:
  explicit LocalPosePublisher(const rclcpp::NodeOptions &node_options);

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cp_pose_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
      sub_goal_nav_sat_fix_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
      sub_cp_nav_sat_fix_;

  // Debug
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pub_raw_local_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_debug_pose_;

private:
  void onGoalNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void
  onCheckpointNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void onDebugPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  boost::optional<geometry_msgs::msg::Pose>
  getClosestPose(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg);

  static geometry_msgs::msg::Point
  geographicCoordinatesToUTM(const double &latitude, const double &longitude,
                             const double &altitude);

  boost::optional<lanelet::LaneletMapPtr>
  getLaneletMap(const double &latitude, const double &longitude,
                const std::string &lanelet2_file_path);

  static void refineAllCenterLines(lanelet::LaneletLayer &lanelet_layer,
                                   const double &center_line_density);

  static boost::optional<geometry_msgs::msg::Pose>
  getClosestCenterLinePoseFromLanelet(
      const lanelet::LaneletLayer &lanelet_layer,
      const geometry_msgs::msg::Point &p, const double &distance_threshold,
      const int &nearest_lanelet_count, const bool &debug_mode);

  void publishPoseStamped(
      const geometry_msgs::msg::Pose &pose,
      const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
          &pub_ptr);

  lanelet::LaneletMapPtr map_;
  geometry_msgs::msg::Point utm_map_origin_;

  double distance_threshold_;
  int nearest_lanelet_count_;
  bool debug_mode_;

  bool goal_ready_;
  int cp_counter_;
};

#endif // LOCAL_POSE_PUBLISHER__LOCAL_POSE_PUBLISHER_HPP_
