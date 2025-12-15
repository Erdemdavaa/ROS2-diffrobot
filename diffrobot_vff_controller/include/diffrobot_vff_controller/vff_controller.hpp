#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/parameter_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace diffrobot_vff_controller
{

class VFFController : public nav2_core::Controller
{
public:
  VFFController() = default;
  ~VFFController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  ) override;



  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav_msgs::msg::Path global_path_;

  // VFF parameters
  double w_attract_{1.0};
  double w_repel_{1.0};
  double lookahead_dist_{0.6};
  double max_lin_{0.25};
  double max_ang_{1.0};
  double repel_radius_{0.8};     // meters
  double robot_radius_{0.18};    // meters (match your costmap)

  // helpers you’ll implement
  geometry_msgs::msg::PoseStamped getLookaheadTarget(const geometry_msgs::msg::PoseStamped & pose);
  std::pair<double,double> computeVFFVector(const geometry_msgs::msg::PoseStamped & pose,
                                            const geometry_msgs::msg::PoseStamped & target);
};

}  // namespace diffrobot_vff_controller
