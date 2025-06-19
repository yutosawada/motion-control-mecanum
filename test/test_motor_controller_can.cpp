#include <gtest/gtest.h>
#include "motion-control-mecanum/motor_controller.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "motion-control-mecanum/motor_constants.hpp"
#include "can/can_interface.hpp"
#include <vector>

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

TEST(MotorControllerCAN, WriteSpeeds) {
  auto mock = std::make_shared<MockCanInterface>();
  motion_control_mecanum::MotorController mc(mock, 1);
  std::array<double,4> speeds{1.0, -2.0, 0.5, 3.0};
  EXPECT_TRUE(mc.writeSpeeds(speeds));
  ASSERT_EQ(mock->sent_frames.size(), 4u);
  for(size_t i=0;i<4;i++) {
    const auto& f = mock->sent_frames[i];
    EXPECT_EQ(f.arbitration_id, 0x200 + i);
    EXPECT_EQ(f.dlc, 4u);
    int32_t val = static_cast<int32_t>(speeds[i] * 1000.0);
    EXPECT_EQ(f.data.size(), 4u);
    int32_t packed = (f.data[0]<<24)|(f.data[1]<<16)|(f.data[2]<<8)|f.data[3];
    EXPECT_EQ(packed, val);
  }
}

TEST(MotorControllerCAN, SetModeOfOperation) {
  auto mock = std::make_shared<MockCanInterface>();
  CanFrame resp;
  resp.arbitration_id = motor_controller::kSdoResponseBaseId + 1;
  resp.dlc = 8;
  resp.data = std::vector<uint8_t>{motor_controller::kSdoExpectedResponseDownload,
                                   0,0,0,0,0,0,0};
  mock->recv_frames.push_back(resp);

  motion_control_mecanum::MotorParameters params{};
  motion_control_mecanum::MotorController mc(mock, 1, params);
  EXPECT_TRUE(mc.SetModeOfOperation(motion_control_mecanum::OperationMode::kProfileVelocity));
  ASSERT_EQ(mock->sent_frames.size(), 1u);
  const auto& f = mock->sent_frames[0];
  EXPECT_EQ(f.arbitration_id, motor_controller::kSdoRequestBaseId + 1);
  EXPECT_EQ(f.dlc, motor_controller::kSdoDlc);
}

