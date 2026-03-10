#include "rclcpp/rclcpp.hpp"
#include "simple_slam/simple_slam_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_slam::SimpleSlamNode>());
  rclcpp::shutdown();
  return 0;
}
