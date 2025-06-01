#include "motion-control-mecanum/motion_controller.hpp"

namespace motion_control_mecanum {

MotionController::MotionController(double wheel_radius, double wheel_separation_x, double wheel_separation_y)
: wheel_radius_(wheel_radius),
  wheel_separation_x_(wheel_separation_x),
  wheel_separation_y_(wheel_separation_y)
{
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

}  // namespace motion_control_mecanum
