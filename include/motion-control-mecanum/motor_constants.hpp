#ifndef ORIENTAL_MOTOR_CONSTANTS_HPP_
#define ORIENTAL_MOTOR_CONSTANTS_HPP_

#include "motion-control-mecanum/motor_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "can/can_interface.hpp"
#include "can/socket_can_interface.hpp"
#include <string>  // std::string 利用のため

namespace motor_controller
{

constexpr uint32_t kSdoRequestBaseId = 0x600;
constexpr uint32_t kSdoResponseBaseId = 0x580;
constexpr size_t kSdoDlc = 8;
constexpr int kReceiveTimeoutMs = 1000;

constexpr uint8_t kSdoExpectedResponseUpload = 0x4B;
constexpr uint8_t kSdoExpectedResponseDownload = 0x60;

constexpr uint8_t kSdoDownload4byteCmd = 0x23;
constexpr uint8_t kSdoDownload3byteCmd = 0x27;
constexpr uint8_t kSdoDownload2byteCmd = 0x2B;
constexpr uint8_t kSdoDownload1byteCmd = 0x2F;

constexpr uint8_t kSdoUploadCmd = 0x40;
constexpr uint8_t kSdoExpectedResponseUpload4byte = 0x43;
constexpr uint8_t kSdoExpectedResponseUpload3byte = 0x47;
constexpr uint8_t kSdoExpectedResponseUpload2byte = 0x4B;
constexpr uint8_t kSdoExpectedResponseUpload1byte = 0x4F;

constexpr uint16_t kFaultResetValue = 0x0080;
constexpr uint16_t kShutdownValue = 0x0006;
constexpr uint16_t kSwitchOnValue = 0x0007;
constexpr uint16_t kEnableOperationValue = 0x000F;
constexpr uint16_t kDisableVoltageValue = 0x0000;
constexpr uint16_t kDisableOperationValue = 0x0007;
constexpr uint16_t kStartHomingOperationValue = 0x001F;
constexpr uint16_t kPostionNewSetPoint = 0x005F;
constexpr uint16_t kPostionChangeSetImmediately = 0x002F;

}

#endif  // ORIENTAL_MOTOR_CONTROLLER_HPP_