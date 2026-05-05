#include "rclcpp/rclcpp.hpp"
#include "sar_rover/rover_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sar_rover::RoverNode>());
  rclcpp::shutdown();
  return 0;
}
