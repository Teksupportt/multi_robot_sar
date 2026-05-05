#include "rclcpp/rclcpp.hpp"
#include "sar_drone/drone_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sar::DroneNode>());
  rclcpp::shutdown();
  return 0;
}
