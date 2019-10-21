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

#include <cstdio>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_msgs/srv/set_message.hpp"

using namespace std::chrono_literals;
using SetMessage = tutorial_msgs::srv::SetMessage;

class Talker : public rclcpp::Node
{
private:
  std::string default_msg_ = "hello parameter!";
  std_msgs::msg::String msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<SetMessage>::SharedPtr srv_;
  std::string decoration_;

public:
  explicit Talker(const std::string & topic_name)
  : Node("talker_with_service_param")
  {
    auto publish_message =
      [this]() -> void
      {
        // decorate by decoration parameter
        this->msg_.data = decoration_ + default_msg_ + decoration_;
        RCLCPP_INFO(this->get_logger(), "%s", this->msg_.data.c_str());
        pub_->publish(this->msg_);
      };

    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    timer_ = this->create_wall_timer(100ms, publish_message);

    auto handle_set_message = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<SetMessage::Request> request,
      std::shared_ptr<SetMessage::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(
          this->get_logger(),
          "message %s -> %s",
          this->msg_.data.c_str(), request->message.c_str()
        );
        this->msg_.data = request->message;
        response->result = true;
      };

    srv_ = this->create_service<SetMessage>("set_message", handle_set_message);

    decoration_ = "";
    // declare parameter: decoration
    this->declare_parameter("decoration");
    // callback function for parameter setting event
    auto parameter_callback =
      [this](const std::vector<rclcpp::Parameter> params)
      -> rcl_interfaces::msg::SetParametersResult
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = false;
        for (auto param : params) {
          if (param.get_name() == "decoration") {
            // set decoration parameter
            decoration_ = param.as_string();
            result.successful = true;
          }
        }
        return result;
      };
    // register parameter setting event callback
    this->set_on_parameters_set_callback(parameter_callback);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Talker>("topic");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
