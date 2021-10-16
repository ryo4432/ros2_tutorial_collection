#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"


class ServiceServer: public rclcpp::Node
{
public:
	ServiceServer():Node("sample_server")
	{
		server_ = create_service<std_srvs::srv::Trigger>(
			"sample_service",
			std::bind(&ServiceServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
	}

	void service_callback(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
	{
		res->success = true;
		res->message = "good";
	}

private:
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServiceServer>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
}
