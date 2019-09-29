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
#include <memory>
#include <thread>
#include "tutorial_msgs/srv/set_message.hpp"
#include "rclcpp/rclcpp.hpp"

using SetMessage = tutorial_msgs::srv::SetMessage;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("service_client");
  auto client = node->create_client<SetMessage>("tutorial_service");
  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }

  auto request = std::make_shared<SetMessage::Request>();
  request->message = "Hello Service! I am Syncronous Service Client.";

  // synchronous call
  auto result_future = client->async_send_request(request);
  // rclcpp::spin_until_future_complete gives a synchronous callback.
  // If this fucntion does not exist, the client become asynchronous callback.
  if(rclcpp::spin_until_future_complete(node, result_future) !=
  rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed.");
    return 1;
  }
  auto response = result_future.get();
  RCLCPP_INFO(node->get_logger(), "result is %s", response->result ? "True" : "False");
  rclcpp::shutdown();
  return 0;
}
