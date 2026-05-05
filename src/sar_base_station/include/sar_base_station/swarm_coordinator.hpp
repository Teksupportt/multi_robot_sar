#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sar_interfaces/msg/target_detection.hpp>
#include <sar_interfaces/msg/swarm_status.hpp>

#include <array>
#include <vector>
#include <string>
#include <map>

namespace sar
{

struct DetectionCluster {
  double x, y, z;
  double confidence;
  uint32_t id;
  int hit_count;  // how many drones detected this cluster
};

class SwarmCoordinator : public rclcpp::Node
{
public:
  explicit SwarmCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  int num_drones_{4};
  double merge_threshold_{1.0};  // meters — detections within this are same target
  double dispatch_timeout_{60.0}; // seconds — dispatch best detection after timeout

  // State
  std::map<uint8_t, std::string> drone_states_;   // drone_id -> state string
  std::vector<DetectionCluster> clusters_;
  bool dispatched_{false};
  rclcpp::Time mission_start_time_;
  bool mission_started_{false};

  // Subscribers — one per drone
  std::vector<rclcpp::Subscription<sar_interfaces::msg::SwarmStatus>::SharedPtr> status_subs_;
  std::vector<rclcpp::Subscription<sar_interfaces::msg::TargetDetection>::SharedPtr> detection_subs_;

  // Publisher — single best detection to rover
  rclcpp::Publisher<sar_interfaces::msg::TargetDetection>::SharedPtr rover_detection_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  // Callbacks
  void statusCallback(const sar_interfaces::msg::SwarmStatus::SharedPtr msg);
  void detectionCallback(const sar_interfaces::msg::TargetDetection::SharedPtr msg, uint8_t drone_id);

  // Logic
  void monitorLoop();
  bool allDronesIdle() const;
  bool allDronesReported() const;
  void mergeDetection(const sar_interfaces::msg::TargetDetection & msg);
  void dispatchBestDetection();
  int findNearestCluster(double x, double y, double z) const;
};

}  // namespace sar
