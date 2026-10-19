#pragma once

#include <memory>
#include <string>
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace apf_controller {

class APFController : public nav2_core::Controller {
public:
  APFController() = default;
  ~APFController() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

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
  struct Force { double fx, fy; };

  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & in_pose,
    const std::string & target_frame);

  void prunePlan(const geometry_msgs::msg::PoseStamped & robot_pose);
  
  Force attractiveForce(const geometry_msgs::msg::PoseStamped & target_in_base);
  Force repulsiveForce();
  
  geometry_msgs::msg::PoseStamped getLookaheadPoint(const geometry_msgs::msg::PoseStamped & robot_pose);

  // Parameters
  double robot_radius_;
  double k_att_;
  double k_rep_;
  double influence_radius_;
  double max_linear_vel_;
  double max_angular_vel_;
  double lookahead_dist_;
  int    lethal_cost_threshold_;

  // Smoothing
  Force last_rep_force_{0.0, 0.0};
  Force last_net_force_{0.0, 0.0};
  double last_linear_vel_ = 0.0;
  double last_angular_vel_ = 0.0;

  size_t pruned_index_{0};

  bool needs_initial_alignment_{false};   // <-- add this
  double initial_align_threshold_;        // <-- add this too if you want the initial-alignment feature from earlier


  // ROS Handles
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("APFController")};
};

}  // namespace apf_controller