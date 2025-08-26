// Copyright (c) 2018 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "nav2_util/node_utils.hpp"
#include "nav2_behaviors/behavior_server.hpp"

namespace behavior_server
{

BehaviorServer::BehaviorServer(const rclcpp::NodeOptions & options)
: LifecycleNode("behavior_server", "", options),
  plugin_loader_("nav2_core", "nav2_core::Behavior"),
  default_ids_{"spin", "backup", "drive_on_heading", "wait"},
  default_types_{"nav2_behaviors::Spin",
    "nav2_behaviors::BackUp",
    "nav2_behaviors::DriveOnHeading",
    "nav2_behaviors::Wait"}
{
  declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
  declare_parameter("behavior_plugins", default_ids_);

  get_parameter("behavior_plugins", behavior_ids_);
  if (behavior_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  declare_parameter(
    "local_frame",
    rclcpp::ParameterValue(std::string("odom")));
  declare_parameter(
    "global_frame",
    rclcpp::ParameterValue(std::string("map")));
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter(
    "transform_tolerance",
    rclcpp::ParameterValue(0.1));
}


BehaviorServer::~BehaviorServer()
{
  behaviors_.clear();
}

nav2_util::CallbackReturn
BehaviorServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  behavior_types_.resize(behavior_ids_.size());
  if (!loadBehaviorPlugins()) {
    on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }
  setupResourcesForBehaviorPlugins();
  configureBehaviorPlugins();

  return nav2_util::CallbackReturn::SUCCESS;
}

bool
BehaviorServer::loadBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    try {
      behavior_types_[i] = nav2_util::get_plugin_type_param(node, behavior_ids_[i]);
      RCLCPP_INFO(
        get_logger(), "Creating behavior plugin %s of type %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      behaviors_.push_back(plugin_loader_.createUniqueInstance(behavior_types_[i]));
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create behavior %s of type %s."
        " Exception: %s", behavior_ids_[i].c_str(), behavior_types_[i].c_str(),
        ex.what());
      return false;
    }
  }

  return true;
}

void BehaviorServer::configureBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    behaviors_[i]->configure(
      node,
      behavior_ids_[i],
      tf_);
  }
}

void BehaviorServer::setupResourcesForBehaviorPlugins()
{
  // std::string robot_base_frame;
  // double transform_tolerance;
  // get_parameter("robot_base_frame", robot_base_frame);
  // transform_tolerance = get_parameter("transform_tolerance").as_double();

  // bool need_local_costmap = false;
  // bool need_global_costmap = false;
  // for (const auto & behavior : behaviors_) {
  //   auto costmap_info = behavior->getResourceInfo();
  //   if (costmap_info == nav2_core::CostmapInfoType::BOTH) {
  //     need_local_costmap = true;
  //     need_global_costmap = true;
  //     break;
  //   }
  //   if (costmap_info == nav2_core::CostmapInfoType::LOCAL) {
  //     need_local_costmap = true;
  //   }
  //   if (costmap_info == nav2_core::CostmapInfoType::GLOBAL) {
  //     need_global_costmap = true;
  //   }
  // }
}

nav2_util::CallbackReturn
BehaviorServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->activate();
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->deactivate();
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->cleanup();
  }

  behaviors_.clear();
  transform_listener_.reset();
  tf_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // end namespace behavior_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_server::BehaviorServer)
