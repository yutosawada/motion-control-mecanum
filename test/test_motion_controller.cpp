#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/motion_controller.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"

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
