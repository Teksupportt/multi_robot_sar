// ~/ros/sar_ws/src/sar_drone/src/drone_node.cpp

#include "sar_drone/drone_node.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace sar
{

// ─── Constants ───────────────────────────────────────────────────────────────
static constexpr float TAKEOFF_ALTITUDE    = -5.0f;   // NED: negative = up
static constexpr float WAYPOINT_THRESHOLD  = 0.5f;
static constexpr float CRUISE_ALTITUDE     = -5.0f;
static constexpr uint8_t ARMING_STATE_ARMED = 2;
static constexpr uint8_t NAV_STATE_OFFBOARD = 14;
static constexpr uint8_t NAV_STATE_AUTO_RTL = 5;
static constexpr uint8_t NAV_STATE_AUTO_LAND = 18;

// ─── Constructor ─────────────────────────────────────────────────────────────
DroneNode::DroneNode(const rclcpp::NodeOptions & options)
: Node("drone_node", options)
{
  this->declare_parameter("drone_id", 0);
  this->declare_parameter("sector_size", 10.0);
  this->declare_parameter("detection_threshold", 1.0);
  this->declare_parameter("test_mode", false);

  drone_id_            = static_cast<uint8_t>(this->get_parameter("drone_id").as_int());
  sector_size_         = this->get_parameter("sector_size").as_double();
  detection_threshold_ = this->get_parameter("detection_threshold").as_double();
  test_mode_           = this->get_parameter("test_mode").as_bool();
  ns_                  = "drone_" + std::to_string(drone_id_);

  buildSectorWaypoints();

  // QoS matching PX4
  auto px4_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);
  auto px4_sub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // PX4 publishers
  offboard_ctrl_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/" + ns_ + "/fmu/in/offboard_control_mode", px4_pub_qos);
  trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/" + ns_ + "/fmu/in/trajectory_setpoint", px4_pub_qos);
  vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/" + ns_ + "/fmu/in/vehicle_command", px4_pub_qos);

  // SAR publishers
  detection_pub_ = this->create_publisher<sar_interfaces::msg::TargetDetection>(
    "/" + ns_ + "/sar/target_detection", reliable_qos);
  swarm_status_pub_ = this->create_publisher<sar_interfaces::msg::SwarmStatus>(
    "/" + ns_ + "/sar/swarm_status", reliable_qos);

  // PX4 subscribers
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/" + ns_ + "/fmu/out/vehicle_status_v4", px4_sub_qos,
    std::bind(&DroneNode::vehicleStatusCallback, this, _1));
  local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/" + ns_ + "/fmu/out/vehicle_local_position_v1", px4_sub_qos,
    std::bind(&DroneNode::localPositionCallback, this, _1));

  // 10 Hz control loop
  control_timer_ = this->create_wall_timer(
    100ms, std::bind(&DroneNode::controlLoop, this));

  // 1 Hz status broadcast
  status_timer_ = this->create_wall_timer(
    1000ms, std::bind(&DroneNode::publishSwarmStatus, this));

  RCLCPP_INFO(this->get_logger(),
    "[%s] DroneNode initialized — sector_size=%.1f detection_threshold=%.1f test_mode=%s",
    ns_.c_str(), sector_size_, detection_threshold_, test_mode_ ? "ON" : "OFF");
}

// ─── Sector / Test Waypoints ──────────────────────────────────────────────────
void DroneNode::buildSectorWaypoints()
{
  if (test_mode_) {
    waypoints_ = {
      {1.0f, 0.0f, CRUISE_ALTITUDE},
      {1.0f, 1.0f, CRUISE_ALTITUDE},
      {0.0f, 1.0f, CRUISE_ALTITUDE},
      {0.0f, 0.0f, CRUISE_ALTITUDE},
    };
    RCLCPP_INFO(this->get_logger(),
      "[%s] TEST MODE — 1m square pattern (offsets applied after position fix)",
      ns_.c_str());
    return;
  }

  float half = static_cast<float>(sector_size_ / 2.0);
  float ox = (drone_id_ % 2) * half;
  float oy = (drone_id_ / 2) * half;

  waypoints_ = {
    {ox,        oy,        CRUISE_ALTITUDE},
    {ox + half, oy,        CRUISE_ALTITUDE},
    {ox + half, oy + half, CRUISE_ALTITUDE},
    {ox,        oy + half, CRUISE_ALTITUDE},
  };

  RCLCPP_INFO(this->get_logger(),
    "[%s] Sector origin: (%.1f, %.1f) size: %.1fx%.1f",
    ns_.c_str(), ox, oy, half, half);
}

// ─── Subscribers ─────────────────────────────────────────────────────────────
void DroneNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  nav_state_    = msg->nav_state;
  arming_state_ = msg->arming_state;
}

void DroneNode::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  current_x_ = msg->x;
  current_y_ = msg->y;
  current_z_ = msg->z;

  if (!position_received_) {
    launch_x_ = current_x_;
    launch_y_ = current_y_;
    launch_z_ = current_z_;
    position_received_ = true;

    RCLCPP_INFO(this->get_logger(),
      "[%s] Launch position saved: [%.2f, %.2f, %.2f]",
      ns_.c_str(), launch_x_, launch_y_, launch_z_);

    if (test_mode_ && !waypoints_offset_) {
      for (auto & wp : waypoints_) {
        wp.x += launch_x_;
        wp.y += launch_y_;
      }
      waypoints_offset_ = true;
      RCLCPP_INFO(this->get_logger(),
        "[%s] Test waypoints offset to launch position (%.2f, %.2f)",
        ns_.c_str(), launch_x_, launch_y_);
    }
  }
}

// ─── Status Broadcast ────────────────────────────────────────────────────────
void DroneNode::publishSwarmStatus()
{
  sar_interfaces::msg::SwarmStatus msg{};
  msg.stamp    = this->get_clock()->now();
  msg.drone_id = drone_id_;
  msg.state    = stateToString(state_);
  msg.pos_x    = static_cast<double>(current_x_);
  msg.pos_y    = static_cast<double>(current_y_);
  msg.pos_z    = static_cast<double>(current_z_);
  swarm_status_pub_->publish(msg);
}

// ─── Main Control Loop ───────────────────────────────────────────────────────
void DroneNode::controlLoop()
{
  if (!position_received_) return;

  if (state_ != DroneState::RTL && state_ != DroneState::LANDED) {
    publishOffboardControlMode();
  }

  offboard_setpoint_counter_++;

  switch (state_) {
    case DroneState::IDLE:                handleIdle();               break;
    case DroneState::OFFBOARD_REQUESTED:  handleOffboardRequested();  break;
    case DroneState::ARMED:               handleArmed();              break;
    case DroneState::TAKEOFF:             handleTakeoff();            break;
    case DroneState::WAYPOINT_FLIGHT:     handleWaypointFlight();     break;
    case DroneState::DETECTION_PUBLISHED: handleDetectionPublished(); break;
    case DroneState::RTL:                 handleRTL();                break;
    case DroneState::LANDED:              handleLanded();             break;
  }
}

// ─── State Handlers ───────────────────────────────────────────────────────────
void DroneNode::handleIdle()
{
  publishTrajectorySetpoint(current_x_, current_y_, current_z_);

  if (offboard_setpoint_counter_ >= 50) {
    RCLCPP_INFO(this->get_logger(), "[%s] Requesting offboard mode", ns_.c_str());
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    state_ = DroneState::OFFBOARD_REQUESTED;
  }
}

void DroneNode::handleOffboardRequested()
{
  publishTrajectorySetpoint(current_x_, current_y_, current_z_);

  if (nav_state_ == NAV_STATE_OFFBOARD) {
    RCLCPP_INFO(this->get_logger(), "[%s] Offboard confirmed — arming", ns_.c_str());
    arm();
    state_ = DroneState::ARMED;
  } else {
    if (offboard_setpoint_counter_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(), "[%s] Retrying offboard mode request", ns_.c_str());
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    }
  }
}

void DroneNode::handleArmed()
{
  publishTrajectorySetpoint(current_x_, current_y_, current_z_);

  if (arming_state_ == ARMING_STATE_ARMED) {
    RCLCPP_INFO(this->get_logger(), "[%s] Armed — taking off", ns_.c_str());
    state_ = DroneState::TAKEOFF;
  }
}

void DroneNode::handleTakeoff()
{
  publishTrajectorySetpoint(launch_x_, launch_y_, TAKEOFF_ALTITUDE);

  if (std::abs(current_z_ - TAKEOFF_ALTITUDE) < WAYPOINT_THRESHOLD) {
    RCLCPP_INFO(this->get_logger(), "[%s] Takeoff complete — beginning search", ns_.c_str());
    current_waypoint_ = 0;
    state_ = DroneState::WAYPOINT_FLIGHT;
  }
}

void DroneNode::handleWaypointFlight()
{
  if (nav_state_ == NAV_STATE_AUTO_RTL) {
    RCLCPP_WARN(this->get_logger(), "[%s] Comm-loss — following PX4 RTL", ns_.c_str());
    state_ = DroneState::RTL;
    return;
  }

  const auto & wp = waypoints_[current_waypoint_];
  publishTrajectorySetpoint(wp.x, wp.y, wp.z);

  if (waypointReached(wp)) {
    RCLCPP_INFO(this->get_logger(), "[%s] Waypoint %zu / %zu reached",
      ns_.c_str(), current_waypoint_ + 1, waypoints_.size());

    current_waypoint_++;

    if (current_waypoint_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "[%s] Search complete — publishing detection", ns_.c_str());
      publishDetection();
      state_ = DroneState::DETECTION_PUBLISHED;
    }
  }
}

void DroneNode::handleDetectionPublished()
{
  RCLCPP_INFO(this->get_logger(), "[%s] Initiating RTL", ns_.c_str());
  triggerRTL();
  state_ = DroneState::RTL;
}

void DroneNode::handleRTL()
{
  float dx = current_x_ - launch_x_;
  float dy = current_y_ - launch_y_;
  float horizontal_dist = std::sqrt(dx*dx + dy*dy);

  if (horizontal_dist < 1.0f) {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  if (nav_state_ == NAV_STATE_AUTO_LAND ||
      arming_state_ != ARMING_STATE_ARMED)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] RTL complete — landed", ns_.c_str());
    state_ = DroneState::LANDED;
  }
}

void DroneNode::handleLanded()
{
  RCLCPP_INFO(this->get_logger(), "[%s] Returning to IDLE", ns_.c_str());
  offboard_setpoint_counter_ = 0;
  current_waypoint_ = 0;
  waypoints_offset_ = false;
  position_received_ = false;
  buildSectorWaypoints();
  state_ = DroneState::IDLE;
}

// ─── PX4 Helpers ─────────────────────────────────────────────────────────────
void DroneNode::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position     = true;
  msg.velocity     = false;
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;
  msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
  offboard_ctrl_pub_->publish(msg);
}

void DroneNode::publishTrajectorySetpoint(float x, float y, float z, float yaw)
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position  = {x, y, z};
  msg.yaw       = yaw;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_pub_->publish(msg);
}

void DroneNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.command          = command;
  msg.param1           = param1;
  msg.param2           = param2;
  msg.target_system    = drone_id_ + 2;
  msg.target_component = 1;
  msg.source_system    = drone_id_ + 2;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_cmd_pub_->publish(msg);
}

void DroneNode::arm()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  RCLCPP_INFO(this->get_logger(), "[%s] Arm command sent", ns_.c_str());
}

void DroneNode::triggerRTL()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 5.0f);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  RCLCPP_INFO(this->get_logger(), "[%s] RTL command sent", ns_.c_str());
}

void DroneNode::publishDetection()
{
  sar_interfaces::msg::TargetDetection msg{};
  msg.stamp        = this->get_clock()->now();
  msg.frame_id     = "map";
  msg.target_x     = static_cast<double>(current_x_);
  msg.target_y     = static_cast<double>(current_y_);
  msg.target_z     = static_cast<double>(current_z_);
  msg.confidence   = 0.99;
  msg.label        = test_mode_ ? "test_target" : "person";
  msg.detection_id = ++detection_id_;
  detection_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "[%s] Detection published — id=%u pos=(%.2f, %.2f, %.2f)",
    ns_.c_str(), detection_id_, msg.target_x, msg.target_y, msg.target_z);
}

// ─── Utility ─────────────────────────────────────────────────────────────────
bool DroneNode::waypointReached(const Waypoint & wp, float threshold) const
{
  float dx = current_x_ - wp.x;
  float dy = current_y_ - wp.y;
  float dz = current_z_ - wp.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz) < threshold;
}

std::string DroneNode::stateToString(DroneState s) const
{
  switch (s) {
    case DroneState::IDLE:                return "IDLE";
    case DroneState::OFFBOARD_REQUESTED:  return "OFFBOARD_REQUESTED";
    case DroneState::ARMED:               return "ARMED";
    case DroneState::TAKEOFF:             return "TAKEOFF";
    case DroneState::WAYPOINT_FLIGHT:     return "WAYPOINT_FLIGHT";
    case DroneState::DETECTION_PUBLISHED: return "DETECTION_PUBLISHED";
    case DroneState::RTL:                 return "RTL";
    case DroneState::LANDED:              return "LANDED";
    default:                              return "UNKNOWN";
  }
}

}  // namespace sar
