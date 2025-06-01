#ifndef MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_

#include <array>
#include <memory>
#include "can/can_interface.hpp"

namespace motion_control_mecanum {

class MotorController {
 public:
  explicit MotorController(std::shared_ptr<can_control::CanInterface> can);

  bool writeSpeeds(const std::array<double, 4> & speeds);

 private:
  std::shared_ptr<can_control::CanInterface> can_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
