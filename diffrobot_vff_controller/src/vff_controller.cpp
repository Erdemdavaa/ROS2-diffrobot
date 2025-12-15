#include "diffrobot_vff_controller/vff_controller.hpp"

#include <cmath>
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



namespace diffrobot_vff_controller
{

void VFFController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)

{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  if (!node_) {
    throw std::runtime_error("Failed to lock node in VFFController");
  }

  // Declare parameters (Humble-compatible)
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".w_attract", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".w_repel", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".repel_radius", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_lin", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_ang", rclcpp::ParameterValue(1.0));

  node_->get_parameter(name_ + ".w_attract", w_attract_);
  node_->get_parameter(name_ + ".w_repel", w_repel_);
  node_->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(name_ + ".repel_radius", repel_radius_);
  node_->get_parameter(name_ + ".max_lin", max_lin_);
  node_->get_parameter(name_ + ".max_ang", max_ang_);

  RCLCPP_INFO(node_->get_logger(), "VFF Controller configured");
}

void VFFController::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "VFF Controller cleanup");
}

void VFFController::activate()
{
  RCLCPP_INFO(node_->get_logger(), "VFF Controller activated");
}

void VFFController::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "VFF Controller deactivated");
}

void VFFController::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
}

geometry_msgs::msg::PoseStamped
VFFController::getLookaheadTarget(const geometry_msgs::msg::PoseStamped & pose)
{
  for (const auto & p : global_path_.poses) {
    double dx = p.pose.position.x - pose.pose.position.x;
    double dy = p.pose.position.y - pose.pose.position.y;
    if (std::hypot(dx, dy) > lookahead_dist_) {
      return p;
    }
  }
  return global_path_.poses.back();
}

geometry_msgs::msg::TwistStamped
VFFController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "base_link";
  double yaw = tf2::getYaw(pose.pose.orientation);


  if (global_path_.poses.empty()) {
    return cmd;
  }

  // ----------------------------
  // 1. Attractive force (goal)
  // ----------------------------
  auto target = getLookaheadTarget(pose);

  double fx = target.pose.position.x - pose.pose.position.x;
  double fy = target.pose.position.y - pose.pose.position.y;

  double goal_dist = std::hypot(fx, fy);
  if (goal_dist > 1e-3) {
    fx /= goal_dist;
    fy /= goal_dist;
  }

  fx *= w_attract_;
  fy *= w_attract_;

  // ----------------------------
  // 2. Repulsive forces (obstacles)
  // ----------------------------
  auto * costmap = costmap_ros_->getCostmap();
  double resolution = costmap->getResolution();

  unsigned int mx, my;
  costmap->worldToMap(
    pose.pose.position.x,
    pose.pose.position.y,
    mx, my);

  int cells = static_cast<int>(repel_radius_ / resolution);

  for (int dx = -cells; dx <= cells; dx++) {
    for (int dy = -cells; dy <= cells; dy++) {
      unsigned int nx = mx + dx;
      unsigned int ny = my + dy;

      if (nx >= costmap->getSizeInCellsX() ||
          ny >= costmap->getSizeInCellsY()) {

            continue;
        }


      unsigned char cost = costmap->getCost(nx, ny);
      if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }

      double wx, wy;
      costmap->mapToWorld(nx, ny, wx, wy);

      double ox = pose.pose.position.x - wx;
      double oy = pose.pose.position.y - wy;

      double d = std::hypot(ox, oy);
      if (d < 1e-3 || d > repel_radius_) {
        continue;
      }

      double rep = w_repel_ * (1.0 / d - 1.0 / repel_radius_) / (d * d);
      fx += rep * (ox / d);
      fy += rep * (oy / d);
    }
  }

  // ----------------------------
  // 3. Convert force (map) → cmd_vel (base_link)
  // ----------------------------

  // Rotate force into base frame
  double fx_base =  std::cos(yaw) * fx + std::sin(yaw) * fy;
  double fy_base = -std::sin(yaw) * fx + std::cos(yaw) * fy;

  // Heading error in base frame
  double heading_error = std::atan2(fy_base, fx_base);

  // Linear velocity: only forward component
  double lin = fx_base;

  // Do NOT drive backwards
  if (lin < 0.0) {
    lin = 0.0;
  }

  cmd.twist.linear.x =
    std::clamp(lin, 0.0, max_lin_);

  cmd.twist.angular.z =
    std::clamp(heading_error, -max_ang_, max_ang_);

return cmd;

}


void VFFController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_lin_ *= speed_limit / 100.0;
  } else {
    max_lin_ = speed_limit;
  }
}

}  // namespace diffrobot_vff_controller

PLUGINLIB_EXPORT_CLASS(
  diffrobot_vff_controller::VFFController,
  nav2_core::Controller)
