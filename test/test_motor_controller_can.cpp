#include <gtest/gtest.h>

#include <vector>

#include "can/can_interface.hpp"
#include "motion-control-mecanum/motor_constants.hpp"
#include "motion-control-mecanum/motor_controller.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"

using can_control::CanFrame;
using can_control::CanInterface;

class MockCanInterface : public CanInterface {
 public:
  std::vector<CanFrame> sent_frames;
  std::vector<CanFrame> recv_frames;
  bool send_ret{true};

  bool Send(const CanFrame& frame) override {
    sent_frames.push_back(frame);
    return send_ret;
  }

  bool Receive(CanFrame* frame, int timeout_ms) override {
    (void)timeout_ms;
    if (recv_frames.empty()) {
      return false;
    }
    *frame = recv_frames.front();
    recv_frames.erase(recv_frames.begin());
    return true;
  }
};

TEST(MotorControllerCAN, SetModeOfOperation) {
  auto mock = std::make_shared<MockCanInterface>();

  motion_control_mecanum::MotorParameters params{};
  motion_control_mecanum::MotorController mc(mock, 1, params);
  mock->sent_frames.clear();
  CanFrame resp;
  resp.arbitration_id = motor_controller::kSdoResponseBaseId + 1;
  resp.dlc = 8;
  resp.data = std::vector<uint8_t>{
      motor_controller::kSdoExpectedResponseDownload, 0, 0, 0, 0, 0, 0, 0};
  mock->recv_frames.push_back(resp);

  EXPECT_TRUE(mc.SetModeOfOperation(
      motion_control_mecanum::OperationMode::kProfileVelocity));
  ASSERT_EQ(mock->sent_frames.size(), 1u);
  const auto& f = mock->sent_frames[0];
  EXPECT_EQ(f.arbitration_id, motor_controller::kSdoRequestBaseId + 1);
  EXPECT_EQ(f.dlc, motor_controller::kSdoDlc);
}

TEST(MotorControllerCAN, SetProfileVelocity) {
  auto mock = std::make_shared<MockCanInterface>();

  motion_control_mecanum::MotorParameters params{};
  motion_control_mecanum::MotorController mc(mock, 1, params);
  mock->sent_frames.clear();
  mock->recv_frames.clear();

  CanFrame resp;
  resp.arbitration_id = motor_controller::kSdoResponseBaseId + 1;
  resp.dlc = 8;
  resp.data = std::vector<uint8_t>{
      motor_controller::kSdoExpectedResponseDownload, 0, 0, 0, 0, 0, 0, 0};
  mock->recv_frames.push_back(resp);

  int32_t velocity = 1234;
  EXPECT_TRUE(mc.SetProfileVelocity(velocity));
  ASSERT_EQ(mock->sent_frames.size(), 1u);
  const auto& f = mock->sent_frames[0];
  EXPECT_EQ(f.arbitration_id, motor_controller::kSdoRequestBaseId + 1);
  EXPECT_EQ(f.dlc, motor_controller::kSdoDlc);
  EXPECT_EQ(f.data[0], motor_controller::kSdoDownload4byteCmd);
  EXPECT_EQ(f.data[1], 0x81);
  EXPECT_EQ(f.data[2], 0x60);
  EXPECT_EQ(f.data[3], 0x00);
  EXPECT_EQ(f.data[4], static_cast<uint8_t>(velocity & 0xFF));
  EXPECT_EQ(f.data[5], static_cast<uint8_t>((velocity >> 8) & 0xFF));
  EXPECT_EQ(f.data[6], static_cast<uint8_t>((velocity >> 16) & 0xFF));
  EXPECT_EQ(f.data[7], static_cast<uint8_t>((velocity >> 24) & 0xFF));
}

TEST(MotorControllerCAN, ReadStatusword) {
  auto mock = std::make_shared<MockCanInterface>();

  motion_control_mecanum::MotorParameters params{};
  motion_control_mecanum::MotorController mc(mock, 1, params);
  mock->sent_frames.clear();
  mock->recv_frames.clear();

  uint16_t status = 0x1234;
  CanFrame resp;
  resp.arbitration_id = motor_controller::kSdoResponseBaseId + 1;
  resp.dlc = 8;
  resp.data = std::vector<uint8_t>{
      motor_controller::kSdoExpectedResponseUpload,
      0, 0, 0,
      static_cast<uint8_t>(status & 0xFF),
      static_cast<uint8_t>((status >> 8) & 0xFF),
      0,
      0};
  mock->recv_frames.push_back(resp);

  uint16_t out_status = 0;
  EXPECT_TRUE(mc.readStatusword(&out_status));
  EXPECT_EQ(out_status, status);

  ASSERT_EQ(mock->sent_frames.size(), 1u);
  const auto& f = mock->sent_frames[0];
  EXPECT_EQ(f.arbitration_id, motor_controller::kSdoRequestBaseId + 1);
  EXPECT_EQ(f.dlc, motor_controller::kSdoDlc);
  EXPECT_EQ(f.data[0], 0x40);
  EXPECT_EQ(f.data[1], 0x41);
  EXPECT_EQ(f.data[2], 0x60);
  EXPECT_EQ(f.data[3], 0x00);
}
