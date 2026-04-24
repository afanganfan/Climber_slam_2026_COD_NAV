// goal_approach_controller.cpp
// Nav2控制器wrapper：在接近目标时限制线速度，防止高速冲过目标点
// 原理：透明代理内部控制器（如MPPI），仅在距目标 < approach_distance 时
//       将合速度钳位到 approach_velocity

#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace goal_approach_controller
{

class GoalApproachController : public nav2_core::Controller
{
public:
  GoalApproachController() = default;
  ~GoalApproachController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();

    // 声明本wrapper的参数
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".inner_plugin",
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_distance",
      rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_velocity",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_distance",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_kp",
      rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_exit_distance",
      rclcpp::ParameterValue(0.55));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_approach_angular_velocity",
      rclcpp::ParameterValue(0.35));

    std::string inner_plugin_type;
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);
    node->get_parameter(name + ".approach_distance", approach_distance_);
    node->get_parameter(name + ".approach_velocity", approach_velocity_);
    node->get_parameter(name + ".direct_approach_distance", direct_approach_distance_);
    node->get_parameter(name + ".direct_approach_kp", direct_approach_kp_);
    node->get_parameter(name + ".direct_approach_exit_distance", direct_approach_exit_distance_);
    node->get_parameter(name + ".max_approach_angular_velocity", max_approach_angular_velocity_);

    tf_ = tf;

    if (direct_approach_distance_ > approach_distance_) {
      RCLCPP_WARN(
        logger_,
        "GoalApproachController: direct_approach_distance(%.2f) > approach_distance(%.2f), "
        "clamping to approach_distance.",
        direct_approach_distance_, approach_distance_);
      direct_approach_distance_ = approach_distance_;
    }

    if (direct_approach_exit_distance_ < direct_approach_distance_) {
      RCLCPP_WARN(
        logger_,
        "GoalApproachController: direct_approach_exit_distance(%.2f) < direct_approach_distance(%.2f), "
        "clamping to direct_approach_distance + 0.1.",
        direct_approach_exit_distance_, direct_approach_distance_);
      direct_approach_exit_distance_ = direct_approach_distance_ + 0.1;
    }

    // 通过pluginlib加载内部控制器
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);
    inner_controller_->configure(parent, name, tf, costmap_ros);

    RCLCPP_INFO(
      logger_,
      "GoalApproachController: 包装 [%s], approach_distance=%.2f m, approach_velocity=%.2f m/s, "
      "direct_approach_distance=%.2f m, direct_approach_exit_distance=%.2f m, "
      "direct_approach_kp=%.2f, max_approach_angular_velocity=%.2f rad/s",
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_,
      direct_approach_distance_, direct_approach_exit_distance_,
      direct_approach_kp_, max_approach_angular_velocity_);
  }

  void cleanup() override
  {
    inner_controller_->cleanup();
  }

  void activate() override
  {
    inner_controller_->activate();
  }

  void deactivate() override
  {
    inner_controller_->deactivate();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
    }
    in_direct_mode_ = false;
    inner_controller_->setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

    if (goal_.header.frame_id.empty()) {
      return cmd;
    }

    geometry_msgs::msg::PoseStamped goal_in_pose_frame = goal_;
    if (goal_.header.frame_id != pose.header.frame_id) {
      try {
        tf_->transform(goal_, goal_in_pose_frame, pose.header.frame_id);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          logger_,
          "GoalApproachController: failed to transform goal from %s to %s: %s",
          goal_.header.frame_id.c_str(), pose.header.frame_id.c_str(), ex.what());
        return cmd;
      }
    }

    // 计算到目标的距离
    double dx = goal_in_pose_frame.pose.position.x - pose.pose.position.x;
    double dy = goal_in_pose_frame.pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (!in_direct_mode_ && dist < direct_approach_distance_) {
      in_direct_mode_ = true;
    } else if (in_direct_mode_ && dist > direct_approach_exit_distance_) {
      in_direct_mode_ = false;
    }

    if (in_direct_mode_) {
      // 近距离直接驱动模式：将目标向量从地图系转换到机体系再下发速度
      // 避免直接使用地图系 dx/dy 导致沿错误方向冲点。
      const auto & q = pose.pose.orientation;
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const double yaw = std::atan2(siny_cosp, cosy_cosp);

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);

      const double dx_body = cos_yaw * dx + sin_yaw * dy;
      const double dy_body = -sin_yaw * dx + cos_yaw * dy;
      const double body_dist = std::hypot(dx_body, dy_body);

      const double target_speed = std::clamp(dist * direct_approach_kp_, 0.0, approach_velocity_);
      if (body_dist > 1e-3) {
        cmd.twist.linear.x = target_speed * (dx_body / body_dist);
        cmd.twist.linear.y = target_speed * (dy_body / body_dist);
      } else {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
      }
      cmd.twist.angular.z = 0.0;
    } else if (dist < approach_distance_) {
      double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
      if (speed > approach_velocity_) {
        double scale = approach_velocity_ / speed;
        cmd.twist.linear.x *= scale;
        cmd.twist.linear.y *= scale;
        // 角速度也按比例降低，避免原地打转
        cmd.twist.angular.z *= scale;
      }

      cmd.twist.angular.z = std::clamp(
        cmd.twist.angular.z,
        -max_approach_angular_velocity_,
        max_approach_angular_velocity_);
    }

    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }

private:
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};
  geometry_msgs::msg::PoseStamped goal_;
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double direct_approach_distance_{0.5};
  double direct_approach_exit_distance_{0.55};
  double direct_approach_kp_{1.0};
  double max_approach_angular_velocity_{0.35};
  bool in_direct_mode_{false};
};

}  // namespace goal_approach_controller

PLUGINLIB_EXPORT_CLASS(
  goal_approach_controller::GoalApproachController,
  nav2_core::Controller)
