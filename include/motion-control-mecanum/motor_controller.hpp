#ifndef MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <vector>
#include <cstdint>
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
  MotorController(std::shared_ptr<can_control::CanInterface> can, uint8_t node_id);

  bool writeSpeeds(const std::array<double, 4> & speeds);

  bool readStatusword(uint16_t * out_status);

  bool FaultReset();
  bool Shutdown();
  bool SwitchOn();
  bool EnableOperation();
  bool DisableVoltage();
  bool DisableOperation();

  // Set the DS402 Modes of Operation (object 0x6060).
  bool SetModeOfOperation(OperationMode mode);

  // Set target velocity in Profile Velocity Mode (object 0x60FF).
  bool SetTargetVelocity(int32_t velocity);

  // Set Velocity window (object 0x606D).
  bool SetVelocityWindow(uint16_t window);

 private:
  bool SendControlWord(uint16_t control_value);
  bool SdoTransaction(const std::vector<uint8_t> & request,
    uint8_t expected_cmd,
    std::vector<uint8_t> & response);

  std::shared_ptr<can_control::CanInterface> can_;
  uint8_t node_id_;
  rclcpp::Logger logger_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
