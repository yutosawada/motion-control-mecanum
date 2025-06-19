#ifndef MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>

#include "can/socket_can_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/motor_controller.hpp"

namespace motion_control_mecanum {

class MotionController {
 public:
  MotionController(double wheel_radius, double wheel_separation_x,
                   double wheel_separation_y);

  MotionController(const std::string& can_device,
                   const std::array<uint8_t, 4>& node_ids, double wheel_radius,
                   double wheel_separation_x, double wheel_separation_y);

  std::array<double, 4> compute(const geometry_msgs::msg::Twist& cmd) const;

  bool writeSpeeds(const std::array<double, 4>& speeds);

  bool servoOn();

  bool servoOff();

 private:
  double wheel_radius_;
  double wheel_separation_x_;
  double wheel_separation_y_;

  std::shared_ptr<can_control::SocketCanInterface> can_interface_;
  std::array<std::shared_ptr<MotorController>, 4> motor_controllers_{};
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTION_CONTROLLER_HPP_
