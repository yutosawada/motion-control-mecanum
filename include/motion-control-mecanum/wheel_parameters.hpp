#ifndef MOTION_CONTROL_MECANUM__WHEEL_PARAMETERS_HPP_
#define MOTION_CONTROL_MECANUM__WHEEL_PARAMETERS_HPP_

namespace motion_control_mecanum {

struct WheelParameters {
  double radius{0.0};
  double separation_x{0.0};
  double separation_y{0.0};
  double gear_ratio{1.0};
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__WHEEL_PARAMETERS_HPP_
