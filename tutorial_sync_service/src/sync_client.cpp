#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class SyncClient : public rclcpp::Node
{
public:
  SyncClient()
  : Node("sync_client")
  {
    callback_group1_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    callback_group2_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);


    client_ = this->create_client<std_srvs::srv::Trigger>("sample_service");

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SyncClient::timer_callback, this),
      callback_group1_);

  }

  void timer_callback()
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client_->async_send_request(req);
    auto status = future.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
      RCLCPP_INFO(get_logger(), "inner srv response is %s", future.get()->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "inner srv future wait failed");
    }
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group2_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SyncClient>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
}
