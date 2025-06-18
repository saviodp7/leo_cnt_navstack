// SPDX-FileCopyrightText: 2024 Salvatore Del Peschio
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <string>
#include "nav2_behavior_tree/plugins/navigate3d_to_pose.hpp"

namespace nav2_behavior_tree
{

Navigate3DToPose::Navigate3DToPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<ActionType>(xml_tag_name, "navigate_3d_to_pose", conf) {}

void Navigate3DToPose::on_tick()
{
  getInput("goal", goal_.target_pose);
  // Altri input se vuoi (es. toleranza, velocità desiderata, ecc.)
}

BT::NodeStatus Navigate3DToPose::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Arrivato al goal 3D con successo");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Navigate3DToPose::on_aborted()
{
  RCLCPP_ERROR(node_->get_logger(), "Navigazione 3D fallita: %s", result_.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Navigate3DToPose::on_cancelled()
{
  RCLCPP_WARN(node_->get_logger(), "Navigazione 3D cancellata");
  return BT::NodeStatus::SUCCESS;
}

void Navigate3DToPose::on_timeout()
{
  RCLCPP_ERROR(node_->get_logger(), "Timeout durante la navigazione 3D");
}

}  // namespace your_bt_package

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<your_bt_package::Navigate3DToPose>(name, config);
    };

  factory.registerBuilder<your_bt_package::Navigate3DToPose>("Navigate3DToPose", builder);
}
