#include <chrono>
#include <cstdio>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "tutorial_msgs/srv/set_message.hpp"

using namespace std::chrono_literals;
using SetMessage = tutorial_msgs::srv::SetMessage;

class Talker : public rclcpp::Node
{
  private:
  std_msgs::msg::String msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<SetMessage>::SharedPtr srv_;
  std::string decoration_;

  public:
  explicit Talker(const std::string &topic_name)
  : Node("talker_with_service_param")
  {
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

    auto publish_message = [this]() -> void
    {
      // decorationによる文字列の装飾
      auto decorated_data = decoration_ + msg_.data + decoration_;
      RCLCPP_INFO(this->get_logger(), "%s", decorated_data.c_str());
      pub_->publish(msg_);
    };

    timer_ = this->create_wall_timer(100ms, publish_message);

    auto handle_set_message = [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<SetMessage::Request> request,
      std::shared_ptr<SetMessage::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "message %s -> %s",
        this->msg_.data.c_str(), request->message.c_str());
        this->msg_.data = request->message;
        response->result = true;
      };

      srv_ = this->create_service<SetMessage>("set_message", handle_set_message);

      decoration_ = "";
      // decorationパラメータの宣言
      this->declare_parameter("decoration");
      // パラメータ設定イベントのコールバック関数
      auto parameter_callback = [this](const std::vector<rclcpp::Parameter> params)->
        rcl_interfaces::msg::SetParametersResult
        {
          auto result = rcl_interfaces::msg::SetParametersResult();
          result.successful = false;
          for(auto param : params)
          {
            if(param.get_name() == "decoration")
            {
              // decorationパラメータの設定
              decoration_ = param.as_string();
              result.successful = true;
            }
          }
          return result;
        };
      // パラメータ設定イベントのコールバック関数設定
      this->set_on_parameters_set_callback(parameter_callback);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Talker>("chatter");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
