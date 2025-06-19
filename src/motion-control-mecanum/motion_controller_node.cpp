#include "motion-control-mecanum/motion_controller_node.hpp"

#include <algorithm>
#include <chrono>

#include "motion-control-mecanum/motor_parameters.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace motion_control_mecanum {

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("motion_controller_node", options) {
  this->declare_parameter("motors.FL_node_id", 1);
  this->declare_parameter("motors.FR_node_id", 2);
  this->declare_parameter("motors.RL_node_id", 3);
  this->declare_parameter("motors.RR_node_id", 4);
  this->declare_parameter("motor_parameters", rclcpp::ParameterValue());
  this->declare_parameter("wheel_parameters", rclcpp::ParameterValue());
  this->declare_parameter("control_parameters", rclcpp::ParameterValue());
  this->declare_parameter("control_parameters.control_frequency", 50.0);
  this->declare_parameter("control_parameters.max_linear_velocity_x", 1.0);
  this->declare_parameter("control_parameters.max_linear_velocity_y", 1.0);
  this->declare_parameter("control_parameters.max_angular_velocity", 1.5);
  
  WheelParameters wheel_params;
  wheel_params.radius =
      this->declare_parameter<double>("wheel_parameters.wheel_radius", 0.075);
  wheel_params.separation_x =
      this->declare_parameter<double>("wheel_parameters.wheel_separation_x", 0.30);
  wheel_params.separation_y =
      this->declare_parameter<double>("wheel_parameters.wheel_separation_y", 0.25);
  
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
  motor_params.quick_stop_deceleration = this->declare_parameter<int>(
      "motor_parameters.quick_stop_deceleration", 1000);
  motor_params.velocity_window =
      this->declare_parameter<int>("motor_parameters.velocity_window", 0);
  motor_params.velocity_threshold =
      this->declare_parameter<int>("motor_parameters.velocity_threshold", 0);

  std::array<uint8_t, 4> node_ids{
      static_cast<uint8_t>(this->get_parameter("motors.FL_node_id").as_int()),
      static_cast<uint8_t>(this->get_parameter("motors.FR_node_id").as_int()),
      static_cast<uint8_t>(this->get_parameter("motors.RL_node_id").as_int()),
      static_cast<uint8_t>(this->get_parameter("motors.RR_node_id").as_int())};
  
  can_interface_ = std::make_shared<can_control::SocketCanInterface>(can_dev);
  motion_controller_ = std::make_shared<MotionController>(
      can_interface_, node_ids, motor_params, wheel_params);

  control_params_.control_frequency =
      this->get_parameter("control_parameters.control_frequency").as_double();
  control_params_.max_linear_velocity_x =
      this->get_parameter("control_parameters.max_linear_velocity_x")
          .as_double();
  control_params_.max_linear_velocity_y =
      this->get_parameter("control_parameters.max_linear_velocity_y")
          .as_double();
  control_params_.max_angular_velocity =
      this->get_parameter("control_parameters.max_angular_velocity")
          .as_double();

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

  motor_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("motor_states", 10);

  publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_params_.control_frequency),
      std::bind(&MotionControllerNode::publishMotorState, this));
}

void MotionControllerNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  geometry_msgs::msg::Twist limited = *msg;
  limited.linear.x =
      std::clamp(limited.linear.x, -control_params_.max_linear_velocity_x,
                 control_params_.max_linear_velocity_x);
  limited.linear.y =
      std::clamp(limited.linear.y, -control_params_.max_linear_velocity_y,
                 control_params_.max_linear_velocity_y);
  limited.angular.z =
      std::clamp(limited.angular.z, -control_params_.max_angular_velocity,
                 control_params_.max_angular_velocity);

  if (motion_controller_->getState() == MotionState::kRunning) {
    (void)motion_controller_->compute(limited);
  }
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

void MotionControllerNode::publishMotorState() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = {"FL", "FR", "RL", "RR"};
  msg.velocity.resize(4);
  msg.effort.resize(4);

  std::array<int16_t, 4> torques{};
  std::array<int32_t, 4> velocities{};
  (void)motion_controller_->getMotorTorques(&torques);
  (void)motion_controller_->getMotorVelocities(&velocities);

  for (size_t i = 0; i < 4; ++i) {
    msg.velocity[i] = static_cast<double>(velocities[i]);
    msg.effort[i] = static_cast<double>(torques[i]);
  }

  motor_state_pub_->publish(msg);
}

}  // namespace motion_control_mecanum
