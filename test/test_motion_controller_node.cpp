#include <gtest/gtest.h>

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "motion-control-mecanum/motion_controller_node.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"

using namespace std::chrono_literals;

class MotionControllerNodeFixture : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
    motion_controller_ = std::make_shared<motion_control_mecanum::MotionController>(wp);
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("control_parameters.control_frequency", 5.0);
    node_ = std::make_shared<motion_control_mecanum::MotionControllerNode>(motion_controller_, opts);
    test_node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(test_node_);
  }

  void TearDown() override {
    executor_->remove_node(node_);
    executor_->remove_node(test_node_);
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<motion_control_mecanum::MotionController> motion_controller_;
  std::shared_ptr<motion_control_mecanum::MotionControllerNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(MotionControllerNodeFixture, ServoOnService) {
  auto client = test_node_->create_client<std_srvs::srv::Trigger>("servo_on");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  auto ret = executor_->spin_until_future_complete(future, 1s);
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(future.get()->success);
  EXPECT_EQ(motion_controller_->getState(), motion_control_mecanum::MotionState::kRunning);
}

TEST_F(MotionControllerNodeFixture, MotorStatePublisher) {
  sensor_msgs::msg::JointState::SharedPtr msg;
  auto sub = test_node_->create_subscription<sensor_msgs::msg::JointState>(
      "motor_states", 10,
      [&](sensor_msgs::msg::JointState::SharedPtr m) { msg = m; });
  (void)sub;
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !msg && (std::chrono::steady_clock::now() - start) < 1s) {
    executor_->spin_once(100ms);
  }
  ASSERT_TRUE(msg != nullptr);
  EXPECT_EQ(msg->name.size(), 4u);
  for (double v : msg->velocity) {
    EXPECT_EQ(v, 0.0);
  }
  for (double e : msg->effort) {
    EXPECT_EQ(e, 0.0);
  }
}

