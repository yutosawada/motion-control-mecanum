#include "motion-control-mecanum/motion_controller_node.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"

namespace motion_control_mecanum {

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("motion_controller_node", options) {
  this->declare_parameter("motors", rclcpp::ParameterValue());
  this->declare_parameter("motor_parameters", rclcpp::ParameterValue());
  this->declare_parameter("wheel_parameters", rclcpp::ParameterValue());
  this->declare_parameter("control_parameters", rclcpp::ParameterValue());

  double radius = this->declare_parameter("wheel_radius", 0.075);
  double sep_x = this->declare_parameter("wheel_separation_x", 0.30);
  double sep_y = this->declare_parameter("wheel_separation_y", 0.25);
  std::string can_dev =
      this->declare_parameter<std::string>("can_device", "can0");

  MotorParameters motor_params;
  motor_params.acceleration =
      this->declare_parameter<int>("motor_parameters.acceleration", 1000);
  motor_params.deceleration =
      this->declare_parameter<int>("motor_parameters.deceleration", 1000);
  motor_params.max_torque =
      this->declare_parameter<int>("motor_parameters.max_torque", 1000);
  motor_params.end_velocity =
      this->declare_parameter<int>("motor_parameters.end_velocity", 0);
  motor_params.quick_stop_deceleration =
      this->declare_parameter<int>("motor_parameters.quick_stop_deceleration",
                                   1000);
  motor_params.velocity_window =
      this->declare_parameter<int>("motor_parameters.velocity_window", 0);
  motor_params.velocity_threshold =
      this->declare_parameter<int>("motor_parameters.velocity_threshold", 0);

  std::array<uint8_t, 4> node_ids{1, 2, 3, 4};
  WheelParameters wheel_params{radius, sep_x, sep_y};
  motion_controller_ = std::make_shared<MotionController>(
      can_dev, node_ids, motor_params, wheel_params);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&MotionControllerNode::cmdVelCallback, this,
                std::placeholders::_1));

  servo_on_service_ = create_service<std_srvs::srv::Trigger>(
      "servo_on", std::bind(&MotionControllerNode::handleServoOn, this,
                            std::placeholders::_1, std::placeholders::_2));

  servo_off_service_ = create_service<std_srvs::srv::Trigger>(
      "servo_off", std::bind(&MotionControllerNode::handleServoOff, this,
                             std::placeholders::_1, std::placeholders::_2));
}

void MotionControllerNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  auto speeds = motion_controller_->compute(*msg);
  motion_controller_->writeSpeeds(speeds);
}

void MotionControllerNode::handleServoOn(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  bool success = motion_controller_->servoOn();
  response->success = success;
  response->message = success ? "servo on" : "servo on failed";
}

void MotionControllerNode::handleServoOff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  bool success = motion_controller_->servoOff();
  response->success = success;
  response->message = success ? "servo off" : "servo off failed";
}

}  // namespace motion_control_mecanum
