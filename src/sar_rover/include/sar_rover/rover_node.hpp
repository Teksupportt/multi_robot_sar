#ifndef SAR_ROVER__ROVER_NODE_HPP_
#define SAR_ROVER__ROVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sar_interfaces/msg/target_detection.hpp"

namespace sar_rover
{
class RoverNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit RoverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void detectionCallback(
    const sar_interfaces::msg::TargetDetection::SharedPtr msg);
  void handleTarget(
    const sar_interfaces::msg::TargetDetection & msg);

  // Nav2 action client callbacks
  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);

  rclcpp::Subscription<sar_interfaces::msg::TargetDetection>::SharedPtr subscription_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  uint32_t last_detection_id_{UINT32_MAX};
};
}  // namespace sar_rover
#endif  // SAR_ROVER__ROVER_NODE_HPP_
