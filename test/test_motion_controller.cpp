#include "motion-control-mecanum/motion_controller.hpp"
#include <gtest/gtest.h>
#include "geometry_msgs/msg/twist.hpp"

TEST(MotionControllerTest, StraightX)
{
  motion_control_mecanum::MotionController mc(0.1, 0.2, 0.2);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  auto speeds = mc.compute(cmd);
  for (double s : speeds) {
    EXPECT_NEAR(s, 10.0, 1e-6);
  }
}

TEST(MotionControllerTest, StraightY)
{
  motion_control_mecanum::MotionController mc(0.1, 0.2, 0.2);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = 1.0;
  auto speeds = mc.compute(cmd);
  EXPECT_NEAR(speeds[0], -10.0, 1e-6);
  EXPECT_NEAR(speeds[1], 10.0, 1e-6);
  EXPECT_NEAR(speeds[2], 10.0, 1e-6);
  EXPECT_NEAR(speeds[3], -10.0, 1e-6);
}
