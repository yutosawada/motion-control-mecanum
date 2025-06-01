#include "motion-control-mecanum/motor_controller.hpp"
#include "motion-control-mecanum/motor_constants.hpp"

namespace motion_control_mecanum {

MotorController::MotorController(std::shared_ptr<can_control::CanInterface> can)
: can_(std::move(can)),
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
  uint8_t node_id,
  const std::vector<uint8_t> & request,
  uint8_t expected_cmd,
  std::vector<uint8_t> & response)
{
  can_control::CanFrame request_frame;
  request_frame.arbitration_id =
    motor_controller::kSdoRequestBaseId + static_cast<uint32_t>(node_id);
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
    (motor_controller::kSdoResponseBaseId + static_cast<uint32_t>(node_id)))
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

bool MotorController::readStatusword(uint8_t node_id, uint16_t * out_status)
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
  if (!SdoTransaction(node_id, request_data,
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
