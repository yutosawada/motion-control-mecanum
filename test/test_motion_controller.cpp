#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "motion-control-mecanum/motion_controller.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"
#include "motion-control-mecanum/motor_parameters.hpp"
#include "can/socket_can_interface.hpp"
#include "motion-control-mecanum/motor_constants.hpp"

using can_control::CanFrame;

class MockSocketCanInterface : public can_control::SocketCanInterface {
 public:
  std::vector<CanFrame> sent_frames;
  std::vector<CanFrame> recv_frames;
  bool send_ret{true};

  MockSocketCanInterface() : SocketCanInterface("can0") {}

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

static CanFrame makeVelocityResp(uint8_t node_id, int32_t vel) {
  CanFrame resp;
  resp.arbitration_id = motor_controller::kSdoResponseBaseId + node_id;
  resp.dlc = 8;
  uint32_t uvel = static_cast<uint32_t>(vel);
  resp.data = {
      motor_controller::kSdoExpectedResponseUpload4byte,
      0, 0, 0,
      static_cast<uint8_t>(uvel & 0xFF),
      static_cast<uint8_t>((uvel >> 8) & 0xFF),
      static_cast<uint8_t>((uvel >> 16) & 0xFF),
      static_cast<uint8_t>((uvel >> 24) & 0xFF)};
  return resp;
}

TEST(MotionControllerTest, StraightX) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(wp);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  EXPECT_TRUE(mc.compute(cmd));
}

TEST(MotionControllerTest, StraightY) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(wp);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = 1.0;
  EXPECT_TRUE(mc.compute(cmd));
}

TEST(MotionControllerTest, ServoOn) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(wp);
  EXPECT_EQ(mc.getState(), motion_control_mecanum::MotionState::kIdle);
  EXPECT_TRUE(mc.servoOn());
  EXPECT_EQ(mc.getState(), motion_control_mecanum::MotionState::kRunning);
}

TEST(MotionControllerTest, ServoOff) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(wp);
  EXPECT_TRUE(mc.servoOn());
  EXPECT_EQ(mc.getState(), motion_control_mecanum::MotionState::kRunning);
  EXPECT_TRUE(mc.servoOff());
  EXPECT_EQ(mc.getState(), motion_control_mecanum::MotionState::kIdle);
}

TEST(MotionControllerTest, GetMotorVelocitiesNoControllers) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(wp);
  std::array<int32_t, 4> velocities{};
  EXPECT_FALSE(mc.getMotorVelocities(&velocities));
  for (const auto v : velocities) {
    EXPECT_EQ(v, 0);
  }
}

TEST(MotionControllerTest, ComputeOdometryNoControllers) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(wp);
  nav_msgs::msg::Odometry odom;
  EXPECT_FALSE(mc.computeOdometry(0.1, &odom));
}

TEST(MotionControllerTest, ComputeOdometry) {
  auto mock_can = std::make_shared<MockSocketCanInterface>();
  motion_control_mecanum::MotorParameters mp{};
  std::array<uint8_t, 4> node_ids{{1, 2, 3, 4}};
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
  motion_control_mecanum::MotionController mc(mock_can, node_ids, mp, wp);

  mock_can->recv_frames.push_back(makeVelocityResp(1, 10));
  mock_can->recv_frames.push_back(makeVelocityResp(2, -10));
  mock_can->recv_frames.push_back(makeVelocityResp(3, -10));
  mock_can->recv_frames.push_back(makeVelocityResp(4, 10));

  nav_msgs::msg::Odometry odom;
  ASSERT_TRUE(mc.computeOdometry(1.0, &odom));
  EXPECT_NEAR(odom.pose.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(odom.pose.pose.orientation.z, 0.0, 1e-6);
  EXPECT_NEAR(odom.pose.pose.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(odom.twist.twist.linear.x, 1.0, 1e-6);
  EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.0, 1e-6);
}
