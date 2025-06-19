#include "motion-control-mecanum/motion_controller_node.hpp"

#include <algorithm>
#include <chrono>

#include "motion-control-mecanum/motor_parameters.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace motion_control_mecanum {
namespace {

void initialize_node(MotionControllerNode * node,
                     std::shared_ptr<MotionController> motion_controller) {
  node->declare_parameter("motors.FL_node_id", 1);
  node->declare_parameter("motors.FR_node_id", 2);
  node->declare_parameter("motors.RL_node_id", 3);
  node->declare_parameter("motors.RR_node_id", 4);
  node->declare_parameter("motor_parameters", rclcpp::ParameterValue());
  node->declare_parameter("wheel_parameters", rclcpp::ParameterValue());
  node->declare_parameter("control_parameters", rclcpp::ParameterValue());
  node->declare_parameter("control_parameters.control_frequency", 50.0);
  node->declare_parameter("control_parameters.max_linear_velocity_x", 1.0);
  node->declare_parameter("control_parameters.max_linear_velocity_y", 1.0);
  node->declare_parameter("control_parameters.max_angular_velocity", 1.5);

  WheelParameters wheel_params;
  wheel_params.radius =
      node->declare_parameter<double>("wheel_parameters.wheel_radius", 0.075);
  wheel_params.separation_x =
      node->declare_parameter<double>("wheel_parameters.wheel_separation_x", 0.30);
  wheel_params.separation_y =
      node->declare_parameter<double>("wheel_parameters.wheel_separation_y", 0.25);

  std::string can_dev = node->declare_parameter<std::string>("can_device", "can0");

  MotorParameters motor_params;
  motor_params.acceleration =
      node->declare_parameter<int>("motor_parameters.acceleration", 1000);
  motor_params.deceleration =
      node->declare_parameter<int>("motor_parameters.deceleration", 1000);
  motor_params.max_torque =
      node->declare_parameter<int>("motor_parameters.max_torque", 1000);
  motor_params.end_velocity =
      node->declare_parameter<int>("motor_parameters.end_velocity", 0);
  motor_params.quick_stop_deceleration = node->declare_parameter<int>(
      "motor_parameters.quick_stop_deceleration", 1000);
  motor_params.velocity_window =
      node->declare_parameter<int>("motor_parameters.velocity_window", 0);
  motor_params.velocity_threshold =
      node->declare_parameter<int>("motor_parameters.velocity_threshold", 0);

  std::array<uint8_t, 4> node_ids{
      static_cast<uint8_t>(node->get_parameter("motors.FL_node_id").as_int()),
      static_cast<uint8_t>(node->get_parameter("motors.FR_node_id").as_int()),
      static_cast<uint8_t>(node->get_parameter("motors.RL_node_id").as_int()),
      static_cast<uint8_t>(node->get_parameter("motors.RR_node_id").as_int())};

  if (!motion_controller) {
    node->can_interface_ =
        std::make_shared<can_control::SocketCanInterface>(can_dev);
    node->motion_controller_ = std::make_shared<MotionController>(
        node->can_interface_, node_ids, motor_params, wheel_params);
  } else {
    node->motion_controller_ = std::move(motion_controller);
  }

  node->control_params_.control_frequency =
      node->get_parameter("control_parameters.control_frequency").as_double();
  node->control_params_.max_linear_velocity_x =
      node->get_parameter("control_parameters.max_linear_velocity_x").as_double();
  node->control_params_.max_linear_velocity_y =
      node->get_parameter("control_parameters.max_linear_velocity_y").as_double();
  node->control_params_.max_angular_velocity =
      node->get_parameter("control_parameters.max_angular_velocity").as_double();

  node->cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&MotionControllerNode::cmdVelCallback, node, std::placeholders::_1));

  node->servo_on_service_ = node->create_service<std_srvs::srv::Trigger>(
      "servo_on", std::bind(&MotionControllerNode::handleServoOn, node,
                            std::placeholders::_1, std::placeholders::_2));

  node->servo_off_service_ = node->create_service<std_srvs::srv::Trigger>(
      "servo_off", std::bind(&MotionControllerNode::handleServoOff, node,
                             std::placeholders::_1, std::placeholders::_2));

  node->motor_state_pub_ =
      node->create_publisher<sensor_msgs::msg::JointState>("motor_states", 10);

  node->publish_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / node->control_params_.control_frequency),
      std::bind(&MotionControllerNode::publishMotorState, node));
}

}  // namespace

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("motion_controller_node", options) {
  initialize_node(this, nullptr);
}

MotionControllerNode::MotionControllerNode(
    std::shared_ptr<MotionController> motion_controller,
    const rclcpp::NodeOptions & options)
    : rclcpp::Node("motion_controller_node", options) {
  initialize_node(this, std::move(motion_controller));
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
