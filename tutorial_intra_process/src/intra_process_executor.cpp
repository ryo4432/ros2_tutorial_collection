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
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Node that produces messages.
class Talker : public rclcpp::Node
{
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit Talker(const std::string & node_name)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    auto msg_ = std_msgs::msg::String();
    msg_.data = "Hello world";

    auto publish_message =
      [this, msg_]() -> void {
        RCLCPP_INFO(this->get_logger(), "%s", msg_.data.c_str());
        this->pub_->publish(msg_);
      };

    timer_ = create_wall_timer(100ms, publish_message);
  }
};

// Node that consumes messages.
class Listener : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

public:
  explicit Listener(const std::string & node_name)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto producer = std::make_shared<Talker>("producer");
  auto consumer = std::make_shared<Listener>("consumer");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();
  rclcpp::spin(producer);

  rclcpp::shutdown();

  return 0;
}
