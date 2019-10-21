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

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

class LifecycleListener : public rclcpp::Node
{
private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_notification_;

  void data_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "dat_callback: %s", msg->data.c_str());
  }

  void notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "notify callback: Transition from state %s to %s",
      msg->start_state.label.c_str(),
      msg->goal_state.label.c_str()
    );
  }

public:
  explicit LifecycleListener(const std::string & node_name)
  : Node(node_name)
  {
    sub_data_ = this->create_subscription<std_msgs::msg::String>(
      "lifecycle_chatter",
      10,
      std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1)
    );

    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/lc_talker/transition_event",
      10,
      std::bind(&LifecycleListener::notification_callback, this, std::placeholders::_1)
    );
  }
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
  rclcpp::spin(lc_listener);
  rclcpp::shutdown();
  return 0;
}
