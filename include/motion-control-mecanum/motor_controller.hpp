#ifndef MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "can/can_interface.hpp"

namespace motion_control_mecanum {

class MotorController {
 public:
  explicit MotorController(std::shared_ptr<can_control::CanInterface> can);

  bool writeSpeeds(const std::array<double, 4> & speeds);

  bool readStatusword(uint8_t node_id, uint16_t * out_status);

 private:
  bool SdoTransaction(uint8_t node_id,
    const std::vector<uint8_t> & request,
    uint8_t expected_cmd,
    std::vector<uint8_t> & response);

  std::shared_ptr<can_control::CanInterface> can_;
  rclcpp::Logger logger_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
