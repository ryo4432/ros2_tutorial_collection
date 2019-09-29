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
