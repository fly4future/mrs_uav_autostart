#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <mutex>
#include <vector>
#include <string>
#include <optional>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test(void);

private:
  rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub_errors_;
  std::mutex                                                        errors_mtx_;
  std::optional<mrs_msgs::msg::ErrorgraphElement>                   last_msg_;
  bool                                                              saw_any_waiting_error_ = false;

  void errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  // Subscribe immediately so we capture the full lifecycle:
  // waiting_for_node errors during startup -> empty errors after startup
  sub_errors_ =
      node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>("/uav1/errors", 100, std::bind(&Tester::errorsCallback, this, std::placeholders::_1));
}

void Tester::errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(errors_mtx_);
  // Only track messages from AutomaticStart — the shared /errors topic carries messages from all managers
  if (msg->source_node.node == "AutomaticStart" && msg->source_node.component == "main") {
    last_msg_ = *msg;

    for (const auto &error : msg->errors) {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
        saw_any_waiting_error_ = true;
        break;
      }
    }
  }
}

bool Tester::test(void) {

  RCLCPP_INFO(node_->get_logger(), "Waiting for errorgraph errors to clear after startup...");

  // AutomaticStart reports waiting_for_node during startup, then stops once ready.
  const int required_consecutive_clean = 3;
  int       consecutive_clean          = 0;

  const auto clean_deadline = node_->get_clock()->now() + rclcpp::Duration(90s);

  while (rclcpp::ok() && node_->get_clock()->now() < clean_deadline && consecutive_clean < required_consecutive_clean) {

    sleep(0.2);

    std::scoped_lock lck(errors_mtx_);
    if (!last_msg_.has_value()) {
      continue;
    }

    bool has_waiting_for_node = false;
    for (const auto &error : last_msg_->errors) {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
        has_waiting_for_node = true;
        break;
      }
    }

    consecutive_clean = has_waiting_for_node ? 0 : consecutive_clean + 1;
  }

  if (consecutive_clean < required_consecutive_clean) {
    RCLCPP_ERROR(node_->get_logger(), "FAILED: errorgraph errors did not clear after startup");
    return false;
  }

  std::scoped_lock lck(errors_mtx_);
  if (!saw_any_waiting_error_) {
    RCLCPP_ERROR(node_->get_logger(), "FAILED: never observed a waiting_for_node error, so clearing proves nothing");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "SUCCESS: errorgraph errors cleared after startup.");
  return true;
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  // Publish the result multiple times to ensure the Python harness receives it,
  // since single-shot publishes can be missed with volatile QoS.
  for (int i = 0; i < 5; i++) {
    tester.reportTestResult(test_result);
    tester.sleep(1.0);
  }

  tester.join();
}
