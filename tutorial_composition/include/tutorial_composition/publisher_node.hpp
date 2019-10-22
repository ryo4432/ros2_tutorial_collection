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

#ifndef TUTORIAL_COMPOSITION__PUBLISHER_NODE_HPP_
#define TUTORIAL_COMPOSITION__PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_composition/visibility.h"

namespace tutorial_composition
{
class PublisherNode : public rclcpp::Node
{
private:
  void on_timer();
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  TUTORIAL_COMPOSITION_PUBLIC
  explicit PublisherNode(rclcpp::NodeOptions options);
};
}  // namespace tutorial_composition

#endif  // TUTORIAL_COMPOSITION__PUBLISHER_NODE_HPP_
