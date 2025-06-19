#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "motion-control-mecanum/motion_controller.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"

TEST(MotionControllerTest, StraightX) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2};
  motion_control_mecanum::MotionController mc(wp);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  EXPECT_TRUE(mc.compute(cmd));
}

TEST(MotionControllerTest, StraightY) {
  motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2};
  motion_control_mecanum::MotionController mc(wp);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.y = 1.0;
  EXPECT_TRUE(mc.compute(cmd));
}
