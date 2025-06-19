#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/motion_controller.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "can/socket_can_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace motion_control_mecanum {

class MotionControllerNode : public rclcpp::Node {
 public:
  explicit MotionControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void handleServoOn(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleServoOff(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servo_on_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servo_off_service_;
  std::shared_ptr<can_control::SocketCanInterface> can_interface_;
  std::shared_ptr<MotionController> motion_controller_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_
