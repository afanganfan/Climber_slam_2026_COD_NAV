#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "chassis");
  this->declare_parameter<std::string>("odom_topic", "Odometry");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "aft_cmd_vel");
  this->declare_parameter<bool>("publish_velocity_marker", true);
  this->declare_parameter<std::string>("velocity_marker_topic", "aft_cmd_vel_direction");
  this->declare_parameter<float>("velocity_marker_scale", 1.2F);
  this->declare_parameter<float>("velocity_marker_max_speed", 1.5F);
  this->declare_parameter<float>("spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("publish_velocity_marker", publish_velocity_marker_);
  this->get_parameter("velocity_marker_topic", velocity_marker_topic_);
  this->get_parameter("velocity_marker_scale", velocity_marker_scale_);
  this->get_parameter("velocity_marker_max_speed", velocity_marker_max_speed_);
  this->get_parameter("spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  if (publish_velocity_marker_) {
    velocity_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(velocity_marker_topic_, 10);
    RCLCPP_INFO(
      get_logger(), "Publish velocity direction marker on: %s", velocity_marker_topic_.c_str());
  }

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 1,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odomCallback, this, std::placeholders::_1));
}

void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

// Transform the velocity from `robot_base_frame` to `fake_robot_base_frame`
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  // Keep command velocity in the controller frame to avoid reversing
  // direction when odometry yaw changes.
  aft_tf_vel.angular.z = msg->angular.z + spin_speed_;
  aft_tf_vel.linear.x = msg->linear.x;
  aft_tf_vel.linear.y = msg->linear.y;

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  publishVelocityMarker(aft_tf_vel);
}

void FakeVelTransform::publishVelocityMarker(const geometry_msgs::msg::Twist & cmd)
{
  if (!publish_velocity_marker_ || !velocity_marker_pub_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = robot_base_frame_;
  marker.ns = "cmd_vel_direction";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.04;
  marker.scale.y = 0.08;
  marker.scale.z = 0.12;
  marker.color.a = 0.95;
  marker.color.r = 0.1;
  marker.color.g = 0.9;
  marker.color.b = 0.3;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  const double vx = static_cast<double>(cmd.linear.x);
  const double vy = static_cast<double>(cmd.linear.y);
  const double speed = std::hypot(vx, vy);

  if (speed < 1e-4) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    velocity_marker_pub_->publish(marker);
    return;
  }

  const double capped_speed =
    std::min(speed, static_cast<double>(velocity_marker_max_speed_));
  const double arrow_length =
    std::max(0.08, capped_speed * static_cast<double>(velocity_marker_scale_));

  geometry_msgs::msg::Point p0;
  p0.x = 0.0;
  p0.y = 0.0;
  p0.z = 0.0;

  geometry_msgs::msg::Point p1;
  p1.x = arrow_length * (vx / speed);
  p1.y = arrow_length * (vy / speed);
  p1.z = 0.0;

  marker.points.push_back(p0);
  marker.points.push_back(p1);
  velocity_marker_pub_->publish(marker);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)