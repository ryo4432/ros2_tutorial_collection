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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
  private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  public:
  explicit LifecycleTalker(const std::string &node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {}

  // callback for walltimer in order to publish the message
  void publish()
  {
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle Hello World #" + std::to_string(++count);

    if (!pub_->is_activated())
    {
      RCLCPP_INFO(this->get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Lifecycle publisher is active. Publishing: [%s]", msg->data.c_str());
    }

    pub_->publish(std::move(msg));
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Transition callback for state configuring
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&LifecycleTalker::publish, this));

    RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

    return CallbackReturn::SUCCESS;
  }

  // Transition callback for state activating
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    pub_->on_activate();
    RCUTILS_LOG_INFO_NAMED(this->get_name(), "on_activate() is called.");
    std::this_thread::sleep_for(2s);

    return CallbackReturn::SUCCESS;
  }

  // Transition callback for state deactivating
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    pub_->on_deactivate();
    RCUTILS_LOG_INFO_NAMED(this->get_name(), "on_deactivate() is called.");

    return CallbackReturn::SUCCESS;
  }

  // Transition callback for state ceaningup
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    pub_.reset();
    RCUTILS_LOG_INFO_NAMED(this->get_name(), "on cleanup is called.");

    return CallbackReturn::SUCCESS;
  }

  // Transition callback for state shutting down
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
  {
    timer_.reset();
    pub_.reset();
    RCUTILS_LOG_INFO_NAMED(this->get_name(), "on shutdown is called from state %s.", state.label().c_str());

    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;
  auto lc_node = std::make_shared<LifecycleTalker>("lc_talker");
  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();
  return 0;
}
