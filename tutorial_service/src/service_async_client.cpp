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
  request->message = "Hello Service! I am Asyncronous Service Client.";

  // asynchronous service call
  using ServiceResponseFuture = rclcpp::Client<SetMessage>::SharedFuture;
  auto response_received_callback = [node](ServiceResponseFuture future){
    auto response = future.get();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(node->get_logger(), "result is %s", response->result ? "True" : "False");
    rclcpp::shutdown();
  };
  auto future_result = client->async_send_request(request, response_received_callback);
  rclcpp::spin(node);// program is shut down not waiting async callback if does not exist this line.
  rclcpp::shutdown();
  return 0;
}
