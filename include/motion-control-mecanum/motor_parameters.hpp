#ifndef MOTION_CONTROL_MECANUM__MOTOR_PARAMETERS_HPP_
#define MOTION_CONTROL_MECANUM__MOTOR_PARAMETERS_HPP_

#include <cstdint>

namespace motion_control_mecanum {

struct MotorParameters {
  int32_t acceleration{0};
  int32_t deceleration{0};
  int32_t max_torque{0};
  int32_t end_velocity{0};
  int32_t quick_stop_deceleration{0};
  int32_t velocity_window{0};
  int32_t velocity_threshold{0};
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTOR_PARAMETERS_HPP_
