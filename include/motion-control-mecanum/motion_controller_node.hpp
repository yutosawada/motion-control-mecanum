#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/motion_controller.hpp"
#include "motion-control-mecanum/motor_controller.hpp"

namespace motion_control_mecanum {

class MotionControllerNode : public rclcpp::Node {
 public:
  explicit MotionControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

 private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<MotionController> motion_controller_;
  std::shared_ptr<MotorController> motor_controller_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_
