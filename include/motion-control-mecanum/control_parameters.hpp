#ifndef MOTION_CONTROL_MECANUM__CONTROL_PARAMETERS_HPP_
#define MOTION_CONTROL_MECANUM__CONTROL_PARAMETERS_HPP_

namespace motion_control_mecanum {

struct ControlParameters {
  double control_frequency{0.0};
  double max_linear_velocity_x{0.0};
  double max_linear_velocity_y{0.0};
  double max_angular_velocity{0.0};
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__CONTROL_PARAMETERS_HPP_
