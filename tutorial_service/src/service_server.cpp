#include <inttypes.h>
#include <memory>
#include "tutorial_msgs/srv/set_message.hpp"
#include "rclcpp/rclcpp.hpp"

using SetMessage = tutorial_msgs::srv::SetMessage;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<SetMessage::Request> request,
  const std::shared_ptr<SetMessage::Response> response
)
{
  (void)request_header; // avoid warning
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %s", request->message.c_str());
    response->result = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("service_server");
  auto server = g_node->create_service<SetMessage>("tutorial_service", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
