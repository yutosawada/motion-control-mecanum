#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include "can/socket_can_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/control_parameters.hpp"
#include "motion-control-mecanum/motion_controller.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace motion_control_mecanum {

class MotionControllerNode : public rclcpp::Node {
 public:
  explicit MotionControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  MotionControllerNode(std::shared_ptr<MotionController> motion_controller,
                       const rclcpp::NodeOptions& options =
                           rclcpp::NodeOptions());

  std::shared_ptr<MotionController> getMotionController() const {
    return motion_controller_;
  }

 private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void handleServoOn(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleServoOff(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publishMotorState();

  void initialize(std::shared_ptr<MotionController> motion_controller);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servo_on_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servo_off_service_;
  std::shared_ptr<can_control::SocketCanInterface> can_interface_;
  std::shared_ptr<MotionController> motion_controller_;
  ControlParameters control_params_{};

  std::string odom_frame_id_{};
  std::string base_frame_id_{};
  std::string odom_topic_name_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Time last_odom_time_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_NODE_HPP_
