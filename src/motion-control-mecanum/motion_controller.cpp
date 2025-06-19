#include "motion-control-mecanum/motion_controller.hpp"

#include "can/socket_can_interface.hpp"

namespace motion_control_mecanum {

MotionController::MotionController(const WheelParameters& wheel_params)
    : wheel_params_(wheel_params), state_(MotionState::kIdle) {}

MotionController::MotionController(
    std::shared_ptr<can_control::SocketCanInterface> can_interface,
    const std::array<uint8_t, 4>& node_ids, const MotorParameters& motor_params,
    const WheelParameters& wheel_params)
    : wheel_params_(wheel_params), state_(MotionState::kIdle),
      can_interface_(std::move(can_interface)) {
  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    motor_controllers_[i] = std::make_shared<MotorController>(
        can_interface_, node_ids[i], motor_params);
  }
}

std::array<double, 4> MotionController::compute(
    const geometry_msgs::msg::Twist& cmd) {
  const double Lx = wheel_params_.separation_x / 2.0;
  const double Ly = wheel_params_.separation_y / 2.0;
  const double k = Lx + Ly;
  const double vx = cmd.linear.x;
  const double vy = cmd.linear.y;
  const double wz = cmd.angular.z;

  std::array<double, 4> speeds{};
  speeds[0] = (vx - vy - k * wz) / wheel_params_.radius;  // front left
  speeds[1] = (vx + vy + k * wz) / wheel_params_.radius;  // front right
  speeds[2] = (vx + vy - k * wz) / wheel_params_.radius;  // rear left
  speeds[3] = (vx - vy + k * wz) / wheel_params_.radius;  // rear right

  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    if (motor_controllers_[i]) {
      motor_controllers_[i]->SetTargetVelocity(
          static_cast<int32_t>(speeds[i]));
    }
  }
  return speeds;
}


bool MotionController::servoOn() {
  bool success = true;
  for (auto& mc : motor_controllers_) {
    if (mc) {
      if (!mc->SwitchOn()) {
        success = false;
      }
      if (!mc->EnableOperation()) {
        success = false;
      }
    }
  }
  if (success) {
    state_ = MotionState::kRunning;
  }
  return success;
}

bool MotionController::servoOff() {
  bool success = true;
  for (auto& mc : motor_controllers_) {
    if (mc) {
      if (!mc->DisableOperation()) {
        success = false;
      }
    }
  }
  if (success) {
    state_ = MotionState::kIdle;
  }
  return success;
}

}  // namespace motion_control_mecanum
