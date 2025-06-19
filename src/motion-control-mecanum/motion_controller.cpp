#include "motion-control-mecanum/motion_controller.hpp"

#include "can/socket_can_interface.hpp"

namespace motion_control_mecanum {

MotionController::MotionController(const WheelParameters& wheel_params)
    : wheel_params_(wheel_params), state_(MotionState::kIdle) {}

MotionController::MotionController(
    std::shared_ptr<can_control::SocketCanInterface> can_interface,
    const std::array<uint8_t, 4>& node_ids, const MotorParameters& motor_params,
    const WheelParameters& wheel_params)
    : wheel_params_(wheel_params),
      state_(MotionState::kIdle),
      can_interface_(std::move(can_interface)) {
  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    motor_controllers_[i] = std::make_shared<MotorController>(
        can_interface_, node_ids[i], motor_params);
  }
}

bool MotionController::compute(const geometry_msgs::msg::Twist& cmd) {
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

  bool success = true;
  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    if (motor_controllers_[i]) {
      if (!motor_controllers_[i]->SetTargetVelocity(
              static_cast<int32_t>(speeds[i]))) {
        success = false;
      }
    }
  }
  return success;
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

bool MotionController::getMotorTorques(
    std::array<int16_t, 4>* out_torques) const {
  bool success = true;
  if (!out_torques) {
    return false;
  }

  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    int16_t torque = 0;
    if (motor_controllers_[i] &&
        motor_controllers_[i]->GetTorqueActualValue(&torque)) {
      (*out_torques)[i] = torque;
    } else {
      success = false;
      (*out_torques)[i] = 0;
    }
  }
  return success;
}

bool MotionController::getMotorVelocities(
    std::array<int32_t, 4>* out_velocities) const {
  bool success = true;
  if (!out_velocities) {
    return false;
  }

  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    int32_t vel = 0;
    if (motor_controllers_[i] &&
        motor_controllers_[i]->GetVelocityActualValue(&vel)) {
      (*out_velocities)[i] = vel;
    } else {
      success = false;
      (*out_velocities)[i] = 0;
    }
  }
  return success;
}

}  // namespace motion_control_mecanum
