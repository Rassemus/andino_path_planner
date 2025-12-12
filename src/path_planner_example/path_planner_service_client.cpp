#include "path_planner_service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace path_planner_example
{

// Implementoi Nav2:n vaatimat rajapinnat (configure, activate, etc.)
void AStarServiceClient::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Alustetaan solmu (node) Nav2:n elinkaarisolmusta
  node_ = parent.lock(); 
  
  // Luo ROS 2 Client, joka kutsuu Python-palvelua /create_plan
  service_client_ = node_->create_client<create_plan_msgs::srv::CreatePlan>("/create_plan");
  RCLCPP_INFO(node_->get_logger(), "AStarServiceClient initialized for service /create_plan.");
}

// Tämä on Nav2:n kutsuma päämetodi, kun maali asetetaan Rviz2:ssa
nav_msgs::msg::Path AStarServiceClient::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // 1. Odotetaan palvelua
  // 2. Täytetään pyyntö start- ja goal-poseilla
  auto request = std::make_shared<create_plan_msgs::srv::CreatePlan::Request>();
  request->start = start;
  request->goal = goal;

  // 3. Lähetetään kutsu Python-palvelimelle ja odotetaan vastausta
  auto future_result = service_client_->async_send_request(request);
  // ... odotuslogiikka ...

  // 4. Palautetaan polku
  return future_result.get()->path;
}

}  // namespace path_planner_example

// TÄRKEÄ: Pluginlib-makro, jotta Nav2 löytää luokan
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_planner_example::AStarServiceClient, nav2_core::GlobalPlanner)