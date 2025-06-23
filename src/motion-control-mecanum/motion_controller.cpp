#include "motion-control-mecanum/motion_controller.hpp"

#include "can/socket_can_interface.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

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

  RCLCPP_DEBUG(logger_, "compute: vx=%.3f vy=%.3f wz=%.3f", vx, vy, wz);

  std::array<double, 4> speeds{};
  speeds[0] = (vx - vy - k * wz) / wheel_params_.radius;  // front left
  speeds[1] = (vx + vy + k * wz) / wheel_params_.radius;  // front right
  speeds[2] = (vx + vy - k * wz) / wheel_params_.radius;  // rear left
  speeds[3] = (vx - vy + k * wz) / wheel_params_.radius;  // rear right

  RCLCPP_DEBUG(logger_, "target speeds: [%.3f, %.3f, %.3f, %.3f]", speeds[0], speeds[1], speeds[2], speeds[3]);

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
  RCLCPP_DEBUG(logger_, "servoOn requested");
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
    RCLCPP_DEBUG(logger_, "servoOn successful");
  }
  return success;
}

bool MotionController::servoOff() {
  RCLCPP_DEBUG(logger_, "servoOff requested");
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
    RCLCPP_DEBUG(logger_, "servoOff successful");
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

bool MotionController::computeOdometry(double dt,
                                       nav_msgs::msg::Odometry* out_odom) {
  if (!out_odom) {
    return false;
  }

  std::array<int32_t, 4> velocities{};
  if (!getMotorVelocities(&velocities)) {
    return false;
  }

  std::array<double, 4> w{};
  for (size_t i = 0; i < w.size(); ++i) {
    w[i] = static_cast<double>(velocities[i]);
  }

  const double Lx = wheel_params_.separation_x / 2.0;
  const double Ly = wheel_params_.separation_y / 2.0;
  const double k = Lx + Ly;

  const double vx = wheel_params_.radius *
                    (w[0] + w[1] + w[2] + w[3]) / 4.0;
  const double vy = wheel_params_.radius *
                    (-w[0] + w[1] + w[2] - w[3]) / 4.0;
  const double wz = wheel_params_.radius *
                    (-w[0] + w[1] - w[2] + w[3]) / (4.0 * k);

  pose_x_ += (vx * std::cos(pose_yaw_) - vy * std::sin(pose_yaw_)) * dt;
  pose_y_ += (vx * std::sin(pose_yaw_) + vy * std::cos(pose_yaw_)) * dt;
  pose_yaw_ += wz * dt;

  RCLCPP_DEBUG(logger_, "odometry: x=%.3f y=%.3f yaw=%.3f", pose_x_, pose_y_, pose_yaw_);

  out_odom->pose.pose.position.x = pose_x_;
  out_odom->pose.pose.position.y = pose_y_;
  out_odom->pose.pose.position.z = 0.0;
  out_odom->pose.pose.orientation.x = 0.0;
  out_odom->pose.pose.orientation.y = 0.0;
  out_odom->pose.pose.orientation.z = std::sin(pose_yaw_ / 2.0);
  out_odom->pose.pose.orientation.w = std::cos(pose_yaw_ / 2.0);

  out_odom->twist.twist.linear.x = vx;
  out_odom->twist.twist.linear.y = vy;
  out_odom->twist.twist.linear.z = 0.0;
  out_odom->twist.twist.angular.x = 0.0;
  out_odom->twist.twist.angular.y = 0.0;
  out_odom->twist.twist.angular.z = wz;

  return true;
}

}  // namespace motion_control_mecanum
