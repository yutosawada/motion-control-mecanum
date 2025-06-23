#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>

#include "can/socket_can_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "motion-control-mecanum/motor_controller.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motion_control_mecanum {

enum class MotionState {
  kIdle,
  kRunning,
  kError,
};

class MotionController {
 public:
  explicit MotionController(const WheelParameters& wheel_params);

  MotionController(
      std::shared_ptr<can_control::SocketCanInterface> can_interface,
      const std::array<uint8_t, 4>& node_ids,
      const MotorParameters& motor_params, const WheelParameters& wheel_params);

  bool compute(const geometry_msgs::msg::Twist& cmd);

  bool servoOn();

  bool servoOff();

  MotionState getState() const { return state_; }

  bool getMotorTorques(std::array<int16_t, 4>* out_torques) const;

  bool getMotorVelocities(std::array<int32_t, 4>* out_velocities) const;

  bool computeOdometry(double dt, nav_msgs::msg::Odometry* out_odom);

 private:
  WheelParameters wheel_params_{};

  MotionState state_{MotionState::kIdle};

  std::shared_ptr<can_control::SocketCanInterface> can_interface_;
  std::array<std::shared_ptr<MotorController>, 4> motor_controllers_{};

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};

  rclcpp::Logger logger_{rclcpp::get_logger("MotionController")};
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
