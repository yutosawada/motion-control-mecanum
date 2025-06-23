#include "motion-control-mecanum/motor_controller.hpp"

#include "motion-control-mecanum/motor_constants.hpp"

namespace motion_control_mecanum {

MotorController::MotorController(std::shared_ptr<can_control::CanInterface> can,
                                 uint8_t node_id)
    : can_(std::move(can)),
      node_id_(node_id),
      motor_params_{},
      logger_(rclcpp::get_logger("MotorController")) {
  if (!ConfigureMotorParameters()) {
    RCLCPP_ERROR(logger_, "Failed to configure motor parameters for node %u",
                 static_cast<unsigned>(node_id_));
  }
}

MotorController::MotorController(std::shared_ptr<can_control::CanInterface> can,
                                 uint8_t node_id, const MotorParameters& params)
    : can_(std::move(can)),
      node_id_(node_id),
      motor_params_(params),
      logger_(rclcpp::get_logger("MotorController")) {
  if (!ConfigureMotorParameters()) {
    RCLCPP_ERROR(logger_, "Failed to configure motor parameters for node %u",
                 static_cast<unsigned>(node_id_));
  }
}

bool MotorController::ConfigureMotorParameters() {
  bool success = true;
  success &= Shutdown();
  success &= SetModeOfOperation(OperationMode::kProfileVelocity);
  success &= SetTargetVelocity(0);
  success &= SetVelocityThreshold(
      static_cast<uint16_t>(motor_params_.velocity_threshold));
  success &=
      SetVelocityWindow(static_cast<uint16_t>(motor_params_.velocity_window));
  success &=
      SetQuickStopOptionCode(QuickStopOptionCode::kQuickStopRampStayQuickStop);
  success &= SetQuickStopDeceleration(
      static_cast<uint32_t>(motor_params_.quick_stop_deceleration));
  success &=
      SetProfileAcceleration(static_cast<uint32_t>(motor_params_.acceleration));
  success &=
      SetProfileDeceleration(static_cast<uint32_t>(motor_params_.deceleration));
  success &= SetEndVelocity(motor_params_.end_velocity);
  success &= SetMaxTorque(static_cast<uint16_t>(motor_params_.max_torque));
  return success;
}

bool MotorController::SdoTransaction(const std::vector<uint8_t>& request,
                                     uint8_t expected_cmd,
                                     std::vector<uint8_t>& response) {
  can_control::CanFrame request_frame;
  request_frame.arbitration_id =
      motor_controller::kSdoRequestBaseId + static_cast<uint32_t>(node_id_);
  request_frame.dlc = motor_controller::kSdoDlc;
  request_frame.data = request;

  RCLCPP_DEBUG(logger_, "SdoTransaction send: id=0x%X", request_frame.arbitration_id);

  if (!can_->Send(request_frame)) {
    RCLCPP_ERROR(logger_, "Failed to send SDO command.");
    return false;
  }

  can_control::CanFrame response_frame;
  if (!can_->Receive(&response_frame, motor_controller::kReceiveTimeoutMs)) {
    RCLCPP_ERROR(logger_, "Failed to receive SDO response.");
    return false;
  }
  if (response_frame.arbitration_id != (motor_controller::kSdoResponseBaseId +
                                        static_cast<uint32_t>(node_id_))) {
    RCLCPP_ERROR(logger_, "Unexpected SDO response CAN ID: 0x%X",
                 response_frame.arbitration_id);
    return false;
  }
  if (response_frame.data.size() < motor_controller::kSdoDlc) {
    RCLCPP_ERROR(logger_, "SDO response data length is insufficient.");
    return false;
  }
  if (response_frame.data[0] != expected_cmd) {
    RCLCPP_ERROR(logger_, "SDO response error: 0x%X", response_frame.data[0]);
    return false;
  }
  response = response_frame.data;
  RCLCPP_DEBUG(logger_, "SdoTransaction success for node %u", static_cast<unsigned>(node_id_));
  return true;
}

bool MotorController::SendControlWord(uint16_t control_value) {
  const uint16_t kControlwordObject = 0x6040;
  const uint8_t kControlwordSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload2byteCmd,
      static_cast<uint8_t>(kControlwordObject & 0xFF),
      static_cast<uint8_t>((kControlwordObject >> 8) & 0xFF),
      kControlwordSubindex,
      static_cast<uint8_t>(control_value & 0xFF),
      static_cast<uint8_t>((control_value >> 8) & 0xFF),
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SendControlWord(%u): Failed to send 0x%04X",
                 static_cast<unsigned>(node_id_), control_value);
    return false;
  }
  RCLCPP_INFO(logger_, "SendControlWord(%u): 0x%04X sent successfully.",
              static_cast<unsigned>(node_id_), control_value);
  return true;
}

bool MotorController::FaultReset() {
  return SendControlWord(motor_controller::kFaultResetValue);
}

bool MotorController::Shutdown() {
  return SendControlWord(motor_controller::kShutdownValue);
}

bool MotorController::SwitchOn() {
  return SendControlWord(motor_controller::kSwitchOnValue);
}

bool MotorController::EnableOperation() {
  return SendControlWord(motor_controller::kEnableOperationValue);
}

bool MotorController::DisableVoltage() {
  return SendControlWord(motor_controller::kDisableVoltageValue);
}

bool MotorController::DisableOperation() {
  return SendControlWord(motor_controller::kDisableOperationValue);
}

bool MotorController::SetModeOfOperation(OperationMode mode) {
  const uint16_t kModeObject = 0x6060;
  const uint8_t kModeSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload1byteCmd,
      static_cast<uint8_t>(kModeObject & 0xFF),
      static_cast<uint8_t>((kModeObject >> 8) & 0xFF),
      kModeSubindex,
      static_cast<uint8_t>(mode),
      0x00,
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetModeOfOperation(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_INFO(logger_, "SetModeOfOperation(%u): mode %d",
              static_cast<unsigned>(node_id_), static_cast<int>(mode));
  return true;
}

bool MotorController::SetTargetVelocity(int32_t velocity) {
  const uint16_t kTargetVelocityObject = 0x60FF;
  const uint8_t kTargetVelocitySubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload4byteCmd,
      static_cast<uint8_t>(kTargetVelocityObject & 0xFF),
      static_cast<uint8_t>((kTargetVelocityObject >> 8) & 0xFF),
      kTargetVelocitySubindex,
      static_cast<uint8_t>(velocity & 0xFF),
      static_cast<uint8_t>((velocity >> 8) & 0xFF),
      static_cast<uint8_t>((velocity >> 16) & 0xFF),
      static_cast<uint8_t>((velocity >> 24) & 0xFF)};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetTargetVelocity(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  return true;
}

bool MotorController::SetVelocityThreshold(uint16_t threshold) {
  const uint16_t kVelocityThresholdObject = 0x606F;
  const uint8_t kVelocityThresholdSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload2byteCmd,
      static_cast<uint8_t>(kVelocityThresholdObject & 0xFF),
      static_cast<uint8_t>((kVelocityThresholdObject >> 8) & 0xFF),
      kVelocityThresholdSubindex,
      static_cast<uint8_t>(threshold & 0xFF),
      static_cast<uint8_t>((threshold >> 8) & 0xFF),
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetVelocityThreshold(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetVelocityThreshold(%u): %u",
              static_cast<unsigned>(node_id_),
              static_cast<unsigned>(threshold));
  return true;
}

bool MotorController::SetVelocityWindow(uint16_t window) {
  const uint16_t kVelocityWindowObject = 0x606D;
  const uint8_t kVelocityWindowSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload2byteCmd,
      static_cast<uint8_t>(kVelocityWindowObject & 0xFF),
      static_cast<uint8_t>((kVelocityWindowObject >> 8) & 0xFF),
      kVelocityWindowSubindex,
      static_cast<uint8_t>(window & 0xFF),
      static_cast<uint8_t>((window >> 8) & 0xFF),
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetVelocityWindow(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetVelocityWindow(%u): %u",
              static_cast<unsigned>(node_id_),
              static_cast<unsigned>(window));
  return true;
}

bool MotorController::SetQuickStopOptionCode(QuickStopOptionCode option) {
  const uint16_t kQuickStopOptionObject = 0x605A;
  const uint8_t kQuickStopOptionSubindex = 0x00;

  int16_t value = static_cast<int16_t>(option);
  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload2byteCmd,
      static_cast<uint8_t>(kQuickStopOptionObject & 0xFF),
      static_cast<uint8_t>((kQuickStopOptionObject >> 8) & 0xFF),
      kQuickStopOptionSubindex,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetQuickStopOptionCode(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetQuickStopOptionCode(%u): %d",
              static_cast<unsigned>(node_id_),
              static_cast<int>(option));
  return true;
}

bool MotorController::SetQuickStopDeceleration(uint32_t deceleration) {
  const uint16_t kQuickStopDecelObject = 0x6085;
  const uint8_t kQuickStopDecelSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload4byteCmd,
      static_cast<uint8_t>(kQuickStopDecelObject & 0xFF),
      static_cast<uint8_t>((kQuickStopDecelObject >> 8) & 0xFF),
      kQuickStopDecelSubindex,
      static_cast<uint8_t>(deceleration & 0xFF),
      static_cast<uint8_t>((deceleration >> 8) & 0xFF),
      static_cast<uint8_t>((deceleration >> 16) & 0xFF),
      static_cast<uint8_t>((deceleration >> 24) & 0xFF)};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetQuickStopDeceleration(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetQuickStopDeceleration(%u): %u",
              static_cast<unsigned>(node_id_),
              static_cast<unsigned>(deceleration));
  return true;
}

bool MotorController::SetProfileAcceleration(uint32_t acceleration) {
  const uint16_t kProfileAccelObject = 0x6083;
  const uint8_t kProfileAccelSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload4byteCmd,
      static_cast<uint8_t>(kProfileAccelObject & 0xFF),
      static_cast<uint8_t>((kProfileAccelObject >> 8) & 0xFF),
      kProfileAccelSubindex,
      static_cast<uint8_t>(acceleration & 0xFF),
      static_cast<uint8_t>((acceleration >> 8) & 0xFF),
      static_cast<uint8_t>((acceleration >> 16) & 0xFF),
      static_cast<uint8_t>((acceleration >> 24) & 0xFF)};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetProfileAcceleration(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetProfileAcceleration(%u): %u",
              static_cast<unsigned>(node_id_),
              static_cast<unsigned>(acceleration));
  return true;
}

bool MotorController::SetProfileDeceleration(uint32_t deceleration) {
  const uint16_t kProfileDecelObject = 0x6084;
  const uint8_t kProfileDecelSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload4byteCmd,
      static_cast<uint8_t>(kProfileDecelObject & 0xFF),
      static_cast<uint8_t>((kProfileDecelObject >> 8) & 0xFF),
      kProfileDecelSubindex,
      static_cast<uint8_t>(deceleration & 0xFF),
      static_cast<uint8_t>((deceleration >> 8) & 0xFF),
      static_cast<uint8_t>((deceleration >> 16) & 0xFF),
      static_cast<uint8_t>((deceleration >> 24) & 0xFF)};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetProfileDeceleration(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetProfileDeceleration(%u): %u",
              static_cast<unsigned>(node_id_),
              static_cast<unsigned>(deceleration));
  return true;
}

bool MotorController::SetEndVelocity(int32_t velocity) {
  const uint16_t kEndVelocityObject = 0x6082;
  const uint8_t kEndVelocitySubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload4byteCmd,
      static_cast<uint8_t>(kEndVelocityObject & 0xFF),
      static_cast<uint8_t>((kEndVelocityObject >> 8) & 0xFF),
      kEndVelocitySubindex,
      static_cast<uint8_t>(velocity & 0xFF),
      static_cast<uint8_t>((velocity >> 8) & 0xFF),
      static_cast<uint8_t>((velocity >> 16) & 0xFF),
      static_cast<uint8_t>((velocity >> 24) & 0xFF)};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetEndVelocity(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetEndVelocity(%u): %d",
              static_cast<unsigned>(node_id_),
              velocity);
  return true;
}

bool MotorController::SetProfileVelocity(int32_t velocity) {
  const uint16_t kProfileVelocityObject = 0x6081;
  const uint8_t kProfileVelocitySubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload4byteCmd,
      static_cast<uint8_t>(kProfileVelocityObject & 0xFF),
      static_cast<uint8_t>((kProfileVelocityObject >> 8) & 0xFF),
      kProfileVelocitySubindex,
      static_cast<uint8_t>(velocity & 0xFF),
      static_cast<uint8_t>((velocity >> 8) & 0xFF),
      static_cast<uint8_t>((velocity >> 16) & 0xFF),
      static_cast<uint8_t>((velocity >> 24) & 0xFF)};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetProfileVelocity(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  return true;
}

bool MotorController::SetMaxTorque(uint16_t max_torque) {
  const uint16_t kMaxTorqueObject = 0x6072;
  const uint8_t kMaxTorqueSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      motor_controller::kSdoDownload2byteCmd,
      static_cast<uint8_t>(kMaxTorqueObject & 0xFF),
      static_cast<uint8_t>((kMaxTorqueObject >> 8) & 0xFF),
      kMaxTorqueSubindex,
      static_cast<uint8_t>(max_torque & 0xFF),
      static_cast<uint8_t>((max_torque >> 8) & 0xFF),
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseDownload,
                      response_data)) {
    RCLCPP_ERROR(logger_, "SetMaxTorque(%u): failed",
                 static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_DEBUG(logger_, "SetMaxTorque(%u): %u",
              static_cast<unsigned>(node_id_),
              static_cast<unsigned>(max_torque));
  return true;
}

bool MotorController::readStatusword(uint16_t* out_status) {
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kStatuswordObject = 0x6041;
  static const uint8_t kStatuswordSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      kSdoUploadRequestCmd,
      static_cast<uint8_t>(kStatuswordObject & 0xFF),
      static_cast<uint8_t>((kStatuswordObject >> 8) & 0xFF),
      kStatuswordSubindex,
      0x00,
      0x00,
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseUpload,
                      response_data)) {
    return false;
  }

  if (response_data.size() < 8) {
    return false;
  }

  uint32_t raw = response_data[4] | (response_data[5] << 8) |
                 (response_data[6] << 16) | (response_data[7] << 24);
  *out_status = static_cast<uint16_t>(raw & 0xFFFF);
  // RCLCPP_INFO(logger_, "Retrieved Statusword: 0x%04X", *out_status);
  return true;
}

bool MotorController::GetTorqueActualValue(int16_t* out_torque) {
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kTorqueActualObject = 0x6077;
  static const uint8_t kTorqueActualSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      kSdoUploadRequestCmd,
      static_cast<uint8_t>(kTorqueActualObject & 0xFF),
      static_cast<uint8_t>((kTorqueActualObject >> 8) & 0xFF),
      kTorqueActualSubindex,
      0x00,
      0x00,
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseUpload,
                      response_data)) {
    return false;
  }

  if (response_data.size() < 8) {
    return false;
  }

  uint32_t raw = response_data[4] | (response_data[5] << 8) |
                 (response_data[6] << 16) | (response_data[7] << 24);
  *out_torque = static_cast<int16_t>(raw & 0xFFFF);
  RCLCPP_DEBUG(logger_, "Torque value for node %u: %d", static_cast<unsigned>(node_id_), *out_torque);
  return true;
}

bool MotorController::GetVelocityActualValue(int32_t* out_velocity) {
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kVelocityActualObject = 0x606C;
  static const uint8_t kVelocityActualSubindex = 0x00;

  std::vector<uint8_t> request_data = {
      kSdoUploadRequestCmd,
      static_cast<uint8_t>(kVelocityActualObject & 0xFF),
      static_cast<uint8_t>((kVelocityActualObject >> 8) & 0xFF),
      kVelocityActualSubindex,
      0x00,
      0x00,
      0x00,
      0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
                      motor_controller::kSdoExpectedResponseUpload,
                      response_data)) {
    return false;
  }

  if (response_data.size() < 8) {
    return false;
  }

  uint32_t raw = response_data[4] | (response_data[5] << 8) |
                 (response_data[6] << 16) | (response_data[7] << 24);
  *out_velocity = static_cast<int32_t>(raw);
  RCLCPP_DEBUG(logger_, "Velocity value for node %u: %d", static_cast<unsigned>(node_id_), *out_velocity);
  return true;
}

}  // namespace motion_control_mecanum
