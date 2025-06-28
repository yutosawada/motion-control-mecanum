#include <gtest/gtest.h>
#include <chrono>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "motion-control-mecanum/motion_controller_node.hpp"
#include "motion-control-mecanum/wheel_parameters.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class MotionControllerNodeFixture : public ::testing::Test {
protected:
  void SetUp() override {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);

    motion_control_mecanum::WheelParameters wp{0.1, 0.2, 0.2, 1.0};
    auto mc = std::make_shared<motion_control_mecanum::MotionController>(wp);

    rclcpp::NodeOptions options;
    options.context(context_);
    options.append_parameter_override("control_parameters.control_frequency", 100.0);

    node_ = std::make_shared<motion_control_mecanum::MotionControllerNode>(mc, options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(context_);
    executor_->add_node(node_);
  }

  void TearDown() override {
    executor_->cancel();
    executor_->remove_node(node_);
    node_.reset();
    executor_.reset();
    context_->shutdown();
    context_.reset();
  }

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<motion_control_mecanum::MotionControllerNode> node_;
};

TEST_F(MotionControllerNodeFixture, ServoOnOffService) {
  auto client_on = node_->create_client<std_srvs::srv::Trigger>("servo_on");
  auto client_off = node_->create_client<std_srvs::srv::Trigger>("servo_off");

  ASSERT_TRUE(client_on->wait_for_service(1s));
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future_on = client_on->async_send_request(req);
  auto ret = executor_->spin_until_future_complete(future_on, 1s);
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future_on.get()->success);
  EXPECT_EQ(node_->getMotionController()->getState(),
            motion_control_mecanum::MotionState::kRunning);

  auto future_off = client_off->async_send_request(req);
  ret = executor_->spin_until_future_complete(future_off, 1s);
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future_off.get()->success);
  EXPECT_EQ(node_->getMotionController()->getState(),
            motion_control_mecanum::MotionState::kIdle);
}

TEST_F(MotionControllerNodeFixture, PublishMotorState) {
  std::promise<sensor_msgs::msg::JointState::SharedPtr> prom;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "motor_states", 10,
      [&prom](sensor_msgs::msg::JointState::SharedPtr msg) {
        prom.set_value(msg);
      });

  auto future_msg = prom.get_future();
  auto status = executor_->spin_until_future_complete(future_msg, 500ms);
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
  auto msg = future_msg.get();
  EXPECT_EQ(msg->name.size(), 4u);
  (void)sub;
}

TEST_F(MotionControllerNodeFixture, NoOdometryWhenComputeFails) {
  std::promise<nav_msgs::msg::Odometry::SharedPtr> prom;
  auto sub = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [&prom](nav_msgs::msg::Odometry::SharedPtr msg) {
        prom.set_value(msg);
      });

  auto future_msg = prom.get_future();
  auto status = executor_->spin_until_future_complete(future_msg, 200ms);
  EXPECT_EQ(status, rclcpp::FutureReturnCode::TIMEOUT);
  (void)sub;
}

