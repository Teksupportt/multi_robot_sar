#include "sar_rover/rover_node.hpp"
#include "rclcpp/qos.hpp"

namespace sar_rover
{
RoverNode::RoverNode(const rclcpp::NodeOptions & options)
: Node("rover_node", options)
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();

  subscription_ = this->create_subscription<sar_interfaces::msg::TargetDetection>(
    "/sar/target_detection",
    qos,
    std::bind(&RoverNode::detectionCallback, this, std::placeholders::_1));

  // Create the Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  RCLCPP_INFO(this->get_logger(),
    "RoverNode started — listening on /sar/target_detection");
}

void RoverNode::detectionCallback(
  const sar_interfaces::msg::TargetDetection::SharedPtr msg)
{
  if (msg->detection_id == last_detection_id_) {
    return;
  }
  last_detection_id_ = msg->detection_id;
  RCLCPP_INFO(this->get_logger(),
    "[%u] Target received: (%.2f, %.2f, %.2f) label='%s' conf=%.2f",
    msg->detection_id,
    msg->target_x, msg->target_y, msg->target_z,
    msg->label.c_str(), msg->confidence);
  handleTarget(*msg);
}

void RoverNode::handleTarget(const sar_interfaces::msg::TargetDetection & msg)
{
  // Check Nav2 is available before sending a goal
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available after 5s");
    return;
  }

  // Build the goal pose from the detection coordinates
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = msg.frame_id;
  goal_msg.pose.pose.position.x = msg.target_x;
  goal_msg.pose.pose.position.y = msg.target_y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.w = 1.0;  // facing forward, no rotation

  RCLCPP_INFO(this->get_logger(),
    "  -> Sending Nav2 goal: (%.2f, %.2f) frame='%s'",
    msg.target_x, msg.target_y, msg.frame_id.c_str());

  // Wire up callbacks and send
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&RoverNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&RoverNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&RoverNode::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void RoverNode::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
  }
}

void RoverNode::feedbackCallback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(),
    "  Distance remaining: %.2f m",
    feedback->distance_remaining);
}

void RoverNode::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Navigation cancelled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown navigation result");
      break;
  }
}

}  // namespace sar_rover
