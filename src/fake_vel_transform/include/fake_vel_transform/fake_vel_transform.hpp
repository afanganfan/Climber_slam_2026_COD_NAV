#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void publishVelocityMarker(const geometry_msgs::msg::Twist & cmd);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_marker_pub_;

  // Broadcast tf from robot_base to robot_base_fake
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string velocity_marker_topic_;
  float spin_speed_;
  bool publish_velocity_marker_{true};
  float velocity_marker_scale_{1.2F};
  float velocity_marker_max_speed_{1.5F};

  double current_robot_base_angle_{0.0};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
