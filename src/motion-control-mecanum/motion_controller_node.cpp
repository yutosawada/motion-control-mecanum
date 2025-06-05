#include "motion-control-mecanum/motion_controller_node.hpp"

namespace motion_control_mecanum {

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("motion_controller_node", options)
{
  this->declare_parameter("motors", rclcpp::ParameterValue());
  this->declare_parameter("motor_parameters", rclcpp::ParameterValue());
  this->declare_parameter("wheel_parameters", rclcpp::ParameterValue());
  this->declare_parameter("control_parameters", rclcpp::ParameterValue());

  double radius = this->declare_parameter("wheel_radius", 0.075);
  double sep_x = this->declare_parameter("wheel_separation_x", 0.30);
  double sep_y = this->declare_parameter("wheel_separation_y", 0.25);
  std::string can_dev = this->declare_parameter<std::string>("can_device", "can0");

  std::array<uint8_t, 4> node_ids{1, 2, 3, 4};
  motion_controller_ = std::make_shared<MotionController>(
    can_dev, node_ids, radius, sep_x, sep_y);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&MotionControllerNode::cmdVelCallback, this, std::placeholders::_1));
}

void MotionControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto speeds = motion_controller_->compute(*msg);
  motion_controller_->writeSpeeds(speeds);
}

}  // namespace motion_control_mecanum
