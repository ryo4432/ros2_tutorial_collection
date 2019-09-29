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
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  Talker(const std::string &node_name, const std::string &topic_name)
  :Node(node_name)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>("talker", "topic"));
  rclcpp::shutdown();

  return 0;
}
