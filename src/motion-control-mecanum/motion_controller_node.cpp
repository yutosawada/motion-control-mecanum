#include "motion-control-mecanum/motion_controller_node.hpp"
#include "can/socket_can_interface.hpp"

namespace motion_control_mecanum {

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("motion_controller_node", options)
{
  double radius = this->declare_parameter("wheel_radius", 0.075);
  double sep_x = this->declare_parameter("wheel_separation_x", 0.30);
  double sep_y = this->declare_parameter("wheel_separation_y", 0.25);
  std::string can_dev = this->declare_parameter<std::string>("can_device", "can0");

  motion_controller_ = std::make_shared<MotionController>(radius, sep_x, sep_y);
  auto can_if = std::make_shared<can_control::SocketCanInterface>(can_dev);
  motor_controller_ = std::make_shared<MotorController>(can_if);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&MotionControllerNode::cmdVelCallback, this, std::placeholders::_1));
}

void MotionControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto speeds = motion_controller_->compute(*msg);
  motor_controller_->writeSpeeds(speeds);
}

}  // namespace motion_control_mecanum
