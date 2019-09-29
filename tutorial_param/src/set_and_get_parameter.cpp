#include <memory>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("set_and_get_parameters");

  // パラメータの宣言
  node->declare_parameter("foo");
  node->declare_parameter("bar");
  node->declare_parameter("baz");

  // パラメータ設定・取得サービスのクライント
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  // パラメータ設定・取得サービスの起動待ち
  while(!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Waitign");
  }

  // パラメータの設定
  auto set_parameters_results = parameters_client->set_parameters(
    {
      rclcpp::Parameter("foo", 2),
      rclcpp::Parameter("bar", "hello"),
      rclcpp::Parameter("baz", 1.45)
    }
  );
  // パラメータ設定成功の確認
  for(auto & result : set_parameters_results)
  {
    if(!result.successful)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed: %s", result.reason.c_str());
    }
  }

  std::stringstream ss;
  // パラメータの取得
  for(auto & parameter : parameters_client->get_parameters({"foo", "bar", "baz"}))
  {
    // パラメータ名とパラメータの型名のロギング
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
  }
  RCLCPP_INFO(node->get_logger(), ss.str().c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}