#include "motion-control-mecanum/motion_controller_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motion_control_mecanum::MotionControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
