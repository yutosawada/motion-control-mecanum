#include "motion-control-mecanum/motion_controller.hpp"
#include "can/socket_can_interface.hpp"

namespace motion_control_mecanum {

MotionController::MotionController(double wheel_radius, double wheel_separation_x, double wheel_separation_y)
: wheel_radius_(wheel_radius),
  wheel_separation_x_(wheel_separation_x),
  wheel_separation_y_(wheel_separation_y)
{
}

MotionController::MotionController(
  const std::string & can_device,
  const std::array<uint8_t, 4> & node_ids,
  double wheel_radius,
  double wheel_separation_x,
  double wheel_separation_y)
: wheel_radius_(wheel_radius),
  wheel_separation_x_(wheel_separation_x),
  wheel_separation_y_(wheel_separation_y)
{
  can_interface_ = std::make_shared<can_control::SocketCanInterface>(can_device);
  for (size_t i = 0; i < motor_controllers_.size(); ++i) {
    motor_controllers_[i] = std::make_shared<MotorController>(can_interface_, node_ids[i]);
  }
}

std::array<double, 4> MotionController::compute(const geometry_msgs::msg::Twist & cmd) const
{
  const double Lx = wheel_separation_x_ / 2.0;
  const double Ly = wheel_separation_y_ / 2.0;
  const double k = Lx + Ly;
  const double vx = cmd.linear.x;
  const double vy = cmd.linear.y;
  const double wz = cmd.angular.z;

  std::array<double, 4> speeds{};
  speeds[0] = (vx - vy - k * wz) / wheel_radius_;  // front left
  speeds[1] = (vx + vy + k * wz) / wheel_radius_;  // front right
  speeds[2] = (vx + vy - k * wz) / wheel_radius_;  // rear left
  speeds[3] = (vx - vy + k * wz) / wheel_radius_;  // rear right
  return speeds;
}

bool MotionController::writeSpeeds(const std::array<double, 4> & speeds)
{
  if (!motor_controllers_.empty() && motor_controllers_[0]) {
    return motor_controllers_[0]->writeSpeeds(speeds);
  }
  return false;
}

bool MotionController::servoOn()
{
  bool success = true;
  for (auto & mc : motor_controllers_) {
    if (mc) {
      if (!mc->SwitchOn()) {
        success = false;
      }
      if (!mc->EnableOperation()) {
        success = false;
      }
    }
  }
  return success;
}

bool MotionController::servoOff()
{
  bool success = true;
  for (auto & mc : motor_controllers_) {
    if (mc) {
      if (!mc->DisableOperation()) {
        success = false;
      }
    }
  }
  return success;
}

}  // namespace motion_control_mecanum
