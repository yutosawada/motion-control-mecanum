#include "motion-control-mecanum/motor_controller.hpp"
#include "motion-control-mecanum/motor_constants.hpp"

namespace motion_control_mecanum {

MotorController::MotorController(std::shared_ptr<can_control::CanInterface> can, uint8_t node_id)
: can_(std::move(can)),
  node_id_(node_id),
  logger_(rclcpp::get_logger("MotorController"))
{
}

bool MotorController::writeSpeeds(const std::array<double, 4> & speeds)
{
  for (size_t i = 0; i < speeds.size(); ++i) {
    can_control::CanFrame frame;
    frame.arbitration_id = 0x200 + static_cast<uint32_t>(i);
    frame.dlc = 4;
    frame.data.resize(4);
    int32_t value = static_cast<int32_t>(speeds[i] * 1000.0);
    frame.data[0] = (value >> 24) & 0xFF;
    frame.data[1] = (value >> 16) & 0xFF;
    frame.data[2] = (value >> 8) & 0xFF;
    frame.data[3] = value & 0xFF;
    if (!can_->Send(frame)) {
      return false;
    }
  }
  return true;
}

bool MotorController::SdoTransaction(
  const std::vector<uint8_t> & request,
  uint8_t expected_cmd,
  std::vector<uint8_t> & response)
{
  can_control::CanFrame request_frame;
  request_frame.arbitration_id =
    motor_controller::kSdoRequestBaseId + static_cast<uint32_t>(node_id_);
  request_frame.dlc = motor_controller::kSdoDlc;
  request_frame.data = request;

  if (!can_->Send(request_frame)) {
    RCLCPP_ERROR(logger_, "Failed to send SDO command.");
    return false;
  }

  can_control::CanFrame response_frame;
  if (!can_->Receive(&response_frame, motor_controller::kReceiveTimeoutMs)) {
    RCLCPP_ERROR(logger_, "Failed to receive SDO response.");
    return false;
  }
  if (response_frame.arbitration_id !=
    (motor_controller::kSdoResponseBaseId + static_cast<uint32_t>(node_id_)))
  {
    RCLCPP_ERROR(logger_,
      "Unexpected SDO response CAN ID: 0x%X", response_frame.arbitration_id);
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
  return true;
}

bool MotorController::SendControlWord(uint16_t control_value)
{
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
      motor_controller::kSdoExpectedResponseDownload, response_data))
  {
    RCLCPP_ERROR(logger_, "SendControlWord(%u): Failed to send 0x%04X",
      static_cast<unsigned>(node_id_), control_value);
    return false;
  }
  RCLCPP_INFO(logger_, "SendControlWord(%u): 0x%04X sent successfully.",
    static_cast<unsigned>(node_id_), control_value);
  return true;
}

bool MotorController::FaultReset()
{
  return SendControlWord(motor_controller::kFaultResetValue);
}

bool MotorController::Shutdown()
{
  return SendControlWord(motor_controller::kShutdownValue);
}

bool MotorController::SwitchOn()
{
  return SendControlWord(motor_controller::kSwitchOnValue);
}

bool MotorController::EnableOperation()
{
  return SendControlWord(motor_controller::kEnableOperationValue);
}

bool MotorController::DisableVoltage()
{
  return SendControlWord(motor_controller::kDisableVoltageValue);
}

bool MotorController::DisableOperation()
{
  return SendControlWord(motor_controller::kDisableOperationValue);
}

bool MotorController::SetModeOfOperation(OperationMode mode)
{
  const uint16_t kModeObject = 0x6060;
  const uint8_t kModeSubindex = 0x00;

  std::vector<uint8_t> request_data = {
    motor_controller::kSdoDownload1byteCmd,
    static_cast<uint8_t>(kModeObject & 0xFF),
    static_cast<uint8_t>((kModeObject >> 8) & 0xFF),
    kModeSubindex,
    static_cast<uint8_t>(mode),
    0x00, 0x00, 0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
      motor_controller::kSdoExpectedResponseDownload, response_data))
  {
    RCLCPP_ERROR(logger_, "SetModeOfOperation(%u): failed", static_cast<unsigned>(node_id_));
    return false;
  }
  RCLCPP_INFO(logger_, "SetModeOfOperation(%u): mode %d", static_cast<unsigned>(node_id_), static_cast<int>(mode));
  return true;
}

bool MotorController::SetTargetVelocity(int32_t velocity)
{
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
      motor_controller::kSdoExpectedResponseDownload, response_data))
  {
    RCLCPP_ERROR(logger_, "SetTargetVelocity(%u): failed", static_cast<unsigned>(node_id_));
    return false;
  }
  return true;
}

bool MotorController::readStatusword(uint16_t * out_status)
{
  static const uint8_t kSdoUploadRequestCmd = 0x40;
  static const uint16_t kStatuswordObject = 0x6041;
  static const uint8_t kStatuswordSubindex = 0x00;

  std::vector<uint8_t> request_data = {
    kSdoUploadRequestCmd,
    static_cast<uint8_t>(kStatuswordObject & 0xFF),
    static_cast<uint8_t>((kStatuswordObject >> 8) & 0xFF),
    kStatuswordSubindex,
    0x00, 0x00, 0x00, 0x00};

  std::vector<uint8_t> response_data;
  if (!SdoTransaction(request_data,
    motor_controller::kSdoExpectedResponseUpload, response_data))
  {
    return false;
  }

  if (response_data.size() < 8) {
    return false;
  }

  uint32_t raw = response_data[4] |
    (response_data[5] << 8) |
    (response_data[6] << 16) |
    (response_data[7] << 24);
  *out_status = static_cast<uint16_t>(raw & 0xFFFF);
  //RCLCPP_INFO(logger_, "Retrieved Statusword: 0x%04X", *out_status);
  return true;
}

}  // namespace motion_control_mecanum
