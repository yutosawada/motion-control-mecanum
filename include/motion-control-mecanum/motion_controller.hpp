#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_

#include <array>
#include "geometry_msgs/msg/twist.hpp"

namespace motion_control_mecanum {

class MotionController {
 public:
  MotionController(double wheel_radius, double wheel_separation_x, double wheel_separation_y);

  std::array<double, 4> compute(const geometry_msgs::msg::Twist & cmd) const;

 private:
  double wheel_radius_;
  double wheel_separation_x_;
  double wheel_separation_y_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
