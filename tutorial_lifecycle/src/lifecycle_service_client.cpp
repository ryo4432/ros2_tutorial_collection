/*
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

// which node to handle
static constexpr char const * lifecycle_node = "lc_talker";
// which service topics to handle
static constexpr char const * node_get_state_topic = "lc_talker/get_state";
static constexpr char const * node_change_state_topic = "lc_talker/change_state";

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleServiceClient : public rclcpp::Node
{
private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

public:
  explicit LifecycleServiceClient(const std::string & node_name)
  : Node(node_name) {}

  void init()
  {
    client_get_state_ =
      this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    client_change_state_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
  }

  // Request the current state of the node
  unsigned int get_state(std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Service %s is not available.",
        client_get_state_->get_service_name()
      );
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current state of the lc_talker node.
    auto future_result = client_get_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we reurn an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Server time out while getting current state for node %s",
        lifecycle_node
      );
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesfull answer. So let's print the current state.
    if (future_result.get()) {
      RCLCPP_INFO(
        this->get_logger(),
        "None %s has current state %s.",
        lifecycle_node,
        future_result.get()->current_state.label.c_str()
      );
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to get current state for node %s",
        lifecycle_node
      );
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  // Invokes a transition
  bool change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Service %s is not available.",
        client_change_state_->get_service_name()
      );
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request);

    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Server time out while getting current state for node %s",
        lifecycle_node
      );
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(
        this->get_logger(),
        "Transition %d successfully triggered.",
        static_cast<int>(transition)
      );
      return true;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to trigger transtion %u",
        static_cast<unsigned int>(transition)
      );
      return false;
    }
  }
};

void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client)
{
  rclcpp::WallRate time_between_state_changes(0.1);  // 10s

  // configure
  {
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
  }

  // deactivate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and deactivate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // cleanup
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // shutdown
  // we have to select TRANSITION_UNCONFIGURED_SHUTDOWN
  // because current state is unconfigured state.
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(
        lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
    {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }
}

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
  lc_client->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<void> script = std::async(
    std::launch::async,
    std::bind(callee_script, lc_client)
  );
  exe.spin_until_future_complete(script);

  rclcpp::shutdown();
  return 0;
}
