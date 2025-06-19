#ifndef MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
#define MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "can/can_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motion_control_mecanum {

enum class OperationMode : int8_t {
  kNoOperation = 0x00,
  kProfilePosition = 0x01,
  kProfileVelocity = 0x03,
  kProfileTorque = 0x04,
  kHoming = 0x06
};

enum class QuickStopOptionCode : int16_t {
  kCustomStopTime = -3,
  kCustomStopRate = -2,
  kImmediateStop = -1,
  kImmediateDisableSwitch = 0,
  kNormalRampDisableSwitch = 1,
  kQuickStopRampDisableSwitch = 2,
  kNormalRampStayQuickStop = 5,
  kQuickStopRampStayQuickStop = 6
};

class MotorController {
 public:
  MotorController(std::shared_ptr<can_control::CanInterface> can,
                  uint8_t node_id);

  bool writeSpeeds(const std::array<double, 4>& speeds);

  bool readStatusword(uint16_t* out_status);

  // Get the actual torque value (object 0x6077).
  bool GetTorqueActualValue(int16_t* out_torque);

  // Get the actual velocity value (object 0x606C).
  bool GetVelocityActualValue(int32_t* out_velocity);

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

  // Set velocity threshold (object 0x606F).
  bool SetVelocityThreshold(uint16_t threshold);

  // Set velocity window (object 0x606D).
  bool SetVelocityWindow(uint16_t window);

  // Set Quick stop option code (object 0x605A).
  bool SetQuickStopOptionCode(QuickStopOptionCode option);

  // Set Quick stop deceleration (object 0x6085).
  bool SetQuickStopDeceleration(uint32_t deceleration);

  // Set profile acceleration (object 0x6083).
  bool SetProfileAcceleration(uint32_t acceleration);

  // Set profile deceleration (object 0x6084).
  bool SetProfileDeceleration(uint32_t deceleration);

  // Set end velocity (object 0x6082).
  bool SetEndVelocity(int32_t velocity);

  // Set profile velocity (object 0x6081).
  bool SetProfileVelocity(int32_t velocity);

  // Set maximum torque limit (object 0x6072).
  bool SetMaxTorque(uint16_t max_torque);

 private:
  bool SendControlWord(uint16_t control_value);
  bool SdoTransaction(const std::vector<uint8_t>& request, uint8_t expected_cmd,
                      std::vector<uint8_t>& response);

  std::shared_ptr<can_control::CanInterface> can_;
  uint8_t node_id_;
  rclcpp::Logger logger_;
};

}  // namespace motion_control_mecanum

#endif  // MOTION_CONTROL_MECANUM__MOTOR_CONTROLLER_HPP_
