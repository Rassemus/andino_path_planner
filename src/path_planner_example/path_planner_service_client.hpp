#ifndef PATH_PLANNER_SERVICE_CLIENT_HPP_
#define PATH_PLANNER_SERVICE_CLIENT_HPP_

#include <nav2_core/global_planner.hpp>
#include <rclcpp/rclcpp.hpp>

#include <create_plan_msgs/srv/create_plan.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace path_planner_example
{

class AStarServiceClient : public nav2_core::GlobalPlanner
{
public:
  AStarServiceClient() = default;
  ~AStarServiceClient() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;

  void deactivate() override;

  void cleanup() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp::Client<create_plan_msgs::srv::CreatePlan>::SharedPtr service_client_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
};

}  // namespace path_planner_example

#endif  // PATH_PLANNER_SERVICE_CLIENT_HPP_