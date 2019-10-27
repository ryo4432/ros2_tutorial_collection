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

#include <memory>
#include <chrono>
#include <regex>
#include <thread>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tutorial_composition/publisher_node.hpp"
#include "tutorial_composition/subscriber_node.hpp"

using namespace std::chrono_literals;

TEST(publisher_node, subscribe)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::string rcv_msg;

  auto pub_node = std::make_shared<tutorial_composition::PublisherNode>(
    rclcpp::NodeOptions().use_intra_process_comms(true)
  );
  auto sub_node = rclcpp::Node::make_shared("subscriber");
  auto subscription = sub_node->create_subscription<std_msgs::msg::String>(
    "topic",
    10,
    [&rcv_msg](std_msgs::msg::String::SharedPtr msg) {
      rcv_msg = msg->data;
    }
  );

  executor.add_node(pub_node);
  executor.add_node(sub_node);
  auto executor_spin_lambda = [&executor]() {
      executor.spin();
    };

  {
    std::thread spin_thread(executor_spin_lambda);
    std::string destinateName = "/publisher_node";
    std::this_thread::sleep_for(2s);

    // node name check
    auto names = sub_node->get_node_graph_interface()->get_node_names();
    bool isFoundName = false;
    for (auto it : names) {
      isFoundName |= it.compare(destinateName) == 0;
    }
    ASSERT_TRUE(isFoundName) << "name is not matched.";

    // publish message check
    bool isMatched = std::regex_match(
      rcv_msg.c_str(),
      std::regex("Hello, world!\\d+"));
    ASSERT_TRUE(isMatched) << "publish message is not matched.";

    executor.cancel();
    spin_thread.join();
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
