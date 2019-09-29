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
