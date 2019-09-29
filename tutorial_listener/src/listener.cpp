#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
  private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  public:
  Listener(const std::string &node_name, const std::string &topic_name):
  Node(node_name)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_name,
      10,
      [this](std_msgs::msg::String::UniquePtr msg){
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>("listener", "topic"));
  rclcpp::shutdown();
  return 0;
}
