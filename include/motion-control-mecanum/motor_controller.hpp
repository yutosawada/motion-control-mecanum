#ifndef MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "can/can_interface.hpp"

namespace motion_control_mecanum {

enum class OperationMode : int8_t {
  kNoOperation = 0x00,
  kProfilePosition = 0x01,
  kProfileVelocity = 0x03,
  kProfileTorque = 0x04,
  kHoming = 0x06
};

class MotorController {
 public:
  explicit MotorController(std::shared_ptr<can_control::CanInterface> can);

  bool writeSpeeds(const std::array<double, 4> & speeds);

  bool readStatusword(uint8_t node_id, uint16_t * out_status);

  bool FaultReset(uint8_t node_id);
  bool Shutdown(uint8_t node_id);
  bool SwitchOn(uint8_t node_id);
  bool EnableOperation(uint8_t node_id);
  bool DisableVoltage(uint8_t node_id);
  bool DisableOperation(uint8_t node_id);

  // Set the DS402 Modes of Operation (object 0x6060).
  bool SetModeOfOperation(uint8_t node_id, OperationMode mode);

  // Set target velocity in Profile Velocity Mode (object 0x60FF).
  bool SetTargetVelocity(uint8_t node_id, int32_t velocity);

 private:
  bool SendControlWord(uint8_t node_id, uint16_t control_value);
  bool SdoTransaction(uint8_t node_id,
    const std::vector<uint8_t> & request,
    uint8_t expected_cmd,
    std::vector<uint8_t> & response);

  std::shared_ptr<can_control::CanInterface> can_;
  rclcpp::Logger logger_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
