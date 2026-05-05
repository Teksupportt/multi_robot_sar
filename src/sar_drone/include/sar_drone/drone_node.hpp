#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sar_interfaces/msg/target_detection.hpp>
#include <sar_interfaces/msg/swarm_status.hpp>

#include <vector>
#include <array>

namespace sar
{

enum class DroneState {
  IDLE,
  OFFBOARD_REQUESTED,
  ARMED,
  TAKEOFF,
  WAYPOINT_FLIGHT,
  DETECTION_PUBLISHED,
  RTL,
  LANDED
};

struct Waypoint {
  float x, y, z;
};

class DroneNode : public rclcpp::Node
{
public:
  explicit DroneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Identity
  uint8_t drone_id_{0};
  std::string ns_;  // e.g. "drone_0"

  // State
  DroneState state_{DroneState::IDLE};
  size_t current_waypoint_{0};
  uint64_t offboard_setpoint_counter_{0};
  uint32_t detection_id_{0};
  float launch_x_{0.0f}, launch_y_{0.0f}, launch_z_{0.0f};
  bool position_received_{false};

  // Sector-based waypoints (set in constructor based on drone_id)
  std::vector<Waypoint> waypoints_;

  // Search parameters (launch-configurable)
  double detection_threshold_;
  double sector_size_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<sar_interfaces::msg::TargetDetection>::SharedPtr detection_pub_;
  rclcpp::Publisher<sar_interfaces::msg::SwarmStatus>::SharedPtr swarm_status_pub_;

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;  // 1Hz status broadcast

  // Callbacks
  void controlLoop();
  void publishSwarmStatus();
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  // State handlers
  void handleIdle();
  void handleOffboardRequested();
  void handleArmed();
  void handleTakeoff();
  void handleWaypointFlight();
  void handleDetectionPublished();
  void handleRTL();
  void handleLanded();

  // PX4 helpers
  void publishOffboardControlMode();
  void publishTrajectorySetpoint(float x, float y, float z, float yaw = 0.0f);
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
  void arm();
  void triggerRTL();

  // Utility
  bool waypointReached(const Waypoint & wp, float threshold = 0.5f) const;
  void publishDetection();
  void buildSectorWaypoints();
  std::string stateToString(DroneState state) const;

  // Cached vehicle state
  uint8_t nav_state_{0};
  uint8_t arming_state_{0};
  float current_x_{0.0f}, current_y_{0.0f}, current_z_{0.0f};
};

}  // namespace sar
