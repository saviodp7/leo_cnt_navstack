#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_controller/controller_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("controller_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating controller server");

  declare_parameter("controller_frequency", 10.0);
  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
}

ControllerServer::~ControllerServer()
{
}

nav2_util::CallbackReturn
ControllerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();
  waypoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/offboard/waypoint", 10);
  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  std::string speed_limit_topic;
  get_parameter("speed_limit_topic", speed_limit_topic);

  // Create the action server that we implement with our followPath method
  // This may throw due to real-time prioritization if user doesn't have real-time permissions
  try {
    action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "follow_path",
      std::bind(&ControllerServer::computeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      true,
      rcl_action_server_get_default_options());
  } 
  catch (const std::runtime_error & e) {
    RCLCPP_ERROR(get_logger(), "Error creating action server! %s", e.what());
    on_cleanup(state);
    return nav2_util::CallbackReturn::FAILURE;
  }

  // Set subscription to the speed limiting topic
  speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10),
    std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();
  auto node = shared_from_this();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Release any allocated resources
  action_server_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  speed_limit_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void ControllerServer::computeControl()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    auto goal = action_server_->get_current_goal();
    if (!goal) return;

    generateTimedTrajectory(goal->path);

    last_valid_cmd_time_ = now();
    rclcpp::WallRate loop_rate(controller_frequency_);
    
    while (rclcpp::ok()) {
      auto start_time = this->now();
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }
      if (action_server_->is_cancel_requested()) {
        action_server_->terminate_all();
        return;
      }
      // Obtain current waypoint from time
      auto current_waypoint = getCurrentWaypoint();
      if (current_waypoint.has_value()) {
          const auto& pose = current_waypoint->pose;
          // Extract yaw from quaternion
          double yaw = tf2::getYaw(pose.orientation);
          
          px4_msgs::msg::TrajectorySetpoint setpoint;
          setpoint.position[0] = pose.position.x;
          setpoint.position[1] = pose.position.y;
          setpoint.position[2] = pose.position.z;
          setpoint.yaw = static_cast<float>(yaw);
          waypoint_pub_->publish(setpoint);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "📍 Setpoint published x=%.2f y=%.2f z=%.2f yaw=%.2f", 
              setpoint.position[0], setpoint.position[1], setpoint.position[2], setpoint.yaw);
      }
      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        current_path_.clear();
        break;
      }
      auto cycle_duration = this->now() - start_time;
      if (!loop_rate.sleep()) {
        RCLCPP_WARN(
            get_logger(),
            "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
            controller_frequency_, 1 / cycle_duration.seconds());
      }
    }
  } catch (nav2_core::ControllerTFError & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::TF_ERROR;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::PatienceExceeded & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::PATIENCE_EXCEEDED;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::InvalidPath & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_PATH;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerTimedOut & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } catch (nav2_core::ControllerException & e) {
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  } 
  catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::UNKNOWN;
    result->error_msg = e.what();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  action_server_->succeeded_current();
}

void ControllerServer::generateTimedTrajectory(const nav_msgs::msg::Path & path)
{
    RCLCPP_DEBUG(get_logger(), "Generating timed trajectory from path");
    if (path.poses.empty()) {
        throw nav2_core::InvalidPath("Path is empty.");
    }
    
    // Reset current path
    current_path_.clear();
    current_waypoint_index_ = 0;
    current_path_.reserve(path.poses.size());
    
    std::vector<geometry_msgs::msg::PoseStamped> path_poses = path.poses;
    
    // Timed trajectory generation with speed limit
    rclcpp::Time start_time = now() + rclcpp::Duration::from_seconds(1.0);
    double accumulated_time = 0.0;
    double current_speed_limit = getSpeedLimit();

    RCLCPP_INFO(get_logger(), "Generating trajectory with speed limit: %.2f m/s", current_speed_limit);

    // First waypoint at the start time
    path_poses[0].header.stamp = start_time;
    current_path_.emplace_back(start_time, path_poses[0]);
   
    // Compute waypoints timing
    for (size_t i = 1; i < path_poses.size(); ++i) {
        const auto& prev_pose = path_poses[i-1].pose;
        const auto& curr_pose = path_poses[i].pose;
        
        // Compute 3D distance and duration
        double dx = curr_pose.position.x - prev_pose.position.x;
        double dy = curr_pose.position.y - prev_pose.position.y;
        double dz = curr_pose.position.z - prev_pose.position.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        double segment_time = distance / current_speed_limit;
        
        // Minimum threshold
        const double min_segment_time = 0.01; // 10ms minimum
        segment_time = std::max(segment_time, min_segment_time);
        accumulated_time += segment_time;
        
        // Set the timestamp
        rclcpp::Time waypoint_time = start_time + rclcpp::Duration::from_seconds(accumulated_time);
        path_poses[i].header.stamp = waypoint_time;
        current_path_.emplace_back(waypoint_time, path_poses[i]);
        
        RCLCPP_DEBUG(get_logger(), 
            "Waypoint %zu: distance=%.3fm, segment_time=%.3fs, total_time=%.3fs", 
            i, distance, segment_time, accumulated_time);
    }
    
    RCLCPP_INFO(get_logger(),
        "Generated %zu timed waypoints with total trajectory time: %.2f seconds (speed: %.2f m/s)",
        current_path_.size(), accumulated_time, current_speed_limit);
}


std::optional<geometry_msgs::msg::PoseStamped> ControllerServer::getCurrentWaypoint()
{
    if (current_path_.empty()) {
        return std::nullopt;
    }
    rclcpp::Time current_time = now();
    // Find current waypoint or increment idx
    while (current_waypoint_index_ < current_path_.size()) {
        const auto& [waypoint_time, pose] = current_path_[current_waypoint_index_];
        if (current_time < waypoint_time) {
            return pose;
        }
        current_waypoint_index_++;
    }
    if (!current_path_.empty()) {
        return current_path_.back().second;
    }
    
    return std::nullopt;
}

// TODO: Pubblicazione feedback
// void ControllerServer::computeAndPublishVelocity()
// {
//   geometry_msgs::msg::PoseStamped pose;
//   std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
//   feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

//   nav_msgs::msg::Path & current_path = current_path_;
//   auto find_closest_pose_idx = [&robot_pose_in_path_frame, &current_path]()
//     {
//       size_t closest_pose_idx = 0;
//       double curr_min_dist = std::numeric_limits<double>::max();
//       for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
//         double curr_dist =
//           nav2_util::geometry_utils::euclidean_distance(robot_pose_in_path_frame,
//           current_path.poses[curr_idx]);
//         if (curr_dist < curr_min_dist) {
//           curr_min_dist = curr_dist;
//           closest_pose_idx = curr_idx;
//         }
//       }
//       return closest_pose_idx;
//     };

//   const std::size_t closest_pose_idx = find_closest_pose_idx();
//   feedback->distance_to_goal = nav2_util::geometry_utils::calculate_path_length(current_path_,
//       closest_pose_idx);
//   action_server_->publish_feedback(feedback);
// }

bool ControllerServer::isGoalReached()
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!tf_buffer_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "TF buffer not initialized");
      return false;
  }
  
  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, "map", "base_link", 0.1)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "❌ Failed to get current pose for goal checking");
      return false;
  }
  
  if (current_path_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "❌ Current path is empty");
      return false;
  }
  
  const auto& [goal_timestamp, goal_pose_stamped] = current_path_.back();
  const auto& query_pose = current_pose.pose;
  const auto& goal_pose = goal_pose_stamped.pose;
  
  double dx = query_pose.position.x - goal_pose.position.x;
  double dy = query_pose.position.y - goal_pose.position.y;
  double dz = query_pose.position.z - goal_pose.position.z;
  
  double distance_sq = dx * dx + dy * dy + dz * dz;
  double distance = std::sqrt(distance_sq);
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                        "XYZ distance to goal: %.3f m (tolerance: %.3f m)", 
                        distance, xyz_goal_tolerance_);
  
  if (distance_sq > xyz_goal_tolerance_) {
      return false;
  }
  
  // We are in position, let's check the orientation
  double current_yaw = tf2::getYaw(query_pose.orientation);
  double goal_yaw = tf2::getYaw(goal_pose.orientation);
  double dyaw = angles::shortest_angular_distance(current_yaw, goal_yaw);
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Yaw difference: %.3f rad (%.1f deg), tolerance: %.3f rad (%.1f deg)", 
                        dyaw, dyaw * 180.0 / M_PI, yaw_goal_tolerance_, yaw_goal_tolerance_ * 180.0 / M_PI);
  bool goal_reached = fabs(dyaw) <= yaw_goal_tolerance_;
  
  return goal_reached;
}

// TODO: Implementazione speed limit via percentuale della velocità massima
void ControllerServer::speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
{
  // Check limit validity or use default
  if (msg->speed_limit > 0.0) {
    speed_limit_ = msg->speed_limit;
    
    RCLCPP_INFO(get_logger(), 
        "Speed limit updated to: %.2f m/s",
        getSpeedLimit());
  }
  else {
    speed_limit_ = default_speed_limit_;
    RCLCPP_WARN(get_logger(), 
        "Received invalid speed limit: %.2f m/s, using default value: %.2f", msg->speed_limit, default_speed_limit_);
  }
}

} // namespace nav2_controller

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_controller::ControllerServer)
