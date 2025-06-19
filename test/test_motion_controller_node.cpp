#include <gtest/gtest.h>

#include "motion-control-mecanum/motion_controller_node.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class MotionControllerNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(MotionControllerNodeTest, ServoOnOff) {
  auto mc = std::make_shared<motion_control_mecanum::MotionController>(
      motion_control_mecanum::WheelParameters{0.1, 0.2, 0.2});
  auto node = std::make_shared<motion_control_mecanum::MotionControllerNode>(mc);

  auto client_on = node->create_client<std_srvs::srv::Trigger>("servo_on");
  auto client_off = node->create_client<std_srvs::srv::Trigger>("servo_off");

  ASSERT_TRUE(client_on->wait_for_service(1s));
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client_on->async_send_request(req);
  rclcpp::spin_until_future_complete(node, future);
  auto resp = future.get();
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(node->getMotionController()->getState(),
            motion_control_mecanum::MotionState::kRunning);

  ASSERT_TRUE(client_off->wait_for_service(1s));
  auto req2 = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future2 = client_off->async_send_request(req2);
  rclcpp::spin_until_future_complete(node, future2);
  auto resp2 = future2.get();
  EXPECT_TRUE(resp2->success);
  EXPECT_EQ(node->getMotionController()->getState(),
            motion_control_mecanum::MotionState::kIdle);
}

