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

#ifndef TUTORIAL_COMPOSITION__SUBSCRIBER_NODE_HPP_
#define TUTORIAL_COMPOSITION__SUBSCRIBER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_composition/visibility.h"

namespace tutorial_composition
{
class SubscriberNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

public:
  TUTORIAL_COMPOSITION_PUBLIC
  explicit SubscriberNode(rclcpp::NodeOptions options);
};
}  // namespace tutorial_composition

#endif  // TUTORIAL_COMPOSITION__SUBSCRIBER_NODE_HPP_
