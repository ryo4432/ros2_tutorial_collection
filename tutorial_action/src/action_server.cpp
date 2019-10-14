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
#include "rclcpp/rclcpp.hpp"
#include "tutorial_msgs/action/fibonacci.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ActionServerNode : public rclcpp::Node
{
  private:
  using Fibonacci = tutorial_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    // Lets's reject sequences that are over 9000
    if(goal->order > 9000)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for(int i=1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Secceeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{
      std::bind(&ActionServerNode::execute, this, _1),
      goal_handle
    }.detach();
  }

  public:
  explicit ActionServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this->get_node_base_interface(), // The node base interface of the corresponding node.
      this->get_node_clock_interface(), // The node clock interface of the corresponding node.
      this->get_node_logging_interface(), // The node logging interface of the corresponding node.
      this->get_node_waitables_interface(), // The node waitables interface of the corresponding node.
      "fibonacci", // action name
      std::bind(&ActionServerNode::handle_goal, this, _1, _2), // GoalCallback: called when goal value is set
      std::bind(&ActionServerNode::handle_cancel, this, _1), // CancelCallback: called when action is cancel
      std::bind(&ActionServerNode::handle_accepted, this, _1 // AcceptedCallback: called when action is executed
      )
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ActionServerNode>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}