#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>

#include "can/socket_can_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/motor_controller.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"

namespace motion_control_mecanum {

class MotionController {
 public:
  explicit MotionController(const WheelParameters& wheel_params);

  MotionController(std::shared_ptr<can_control::SocketCanInterface> can_interface,
                   const std::array<uint8_t, 4>& node_ids,
                   const MotorParameters& motor_params,
                   const WheelParameters& wheel_params);

  std::array<double, 4> compute(const geometry_msgs::msg::Twist& cmd) const;


  bool servoOn();

  bool servoOff();

 private:
  WheelParameters wheel_params_{};

  std::shared_ptr<can_control::SocketCanInterface> can_interface_;
  std::array<std::shared_ptr<MotorController>, 4> motor_controllers_{};
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
