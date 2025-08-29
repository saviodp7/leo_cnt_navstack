#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_planner/planner_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("expected_planner_frequency", 1.0);
  declare_parameter("action_server_result_timeout", 10.0);
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Initialize TF2
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  double action_server_result_timeout;
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create client for planning service
  planning_client_ = create_client<x500_trajectory_planner::srv::X500PlanningService>(
  "x500_planner");

  // Create the action servers for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true, server_options);

  // TODO: Planner ActionServerThroughPoses
  // action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
  //   shared_from_this(),
  //   "compute_path_through_poses",
  //   std::bind(&PlannerServer::computePlanThroughPoses, this),
  //   nullptr,
  //   std::chrono::milliseconds(500),
  //   true, server_options);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_pose_->activate();
  // action_server_poses_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  // action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  // action_server_poses_.reset();
  plan_publisher_.reset();
  planning_client_.reset();
  tf_listener_.reset();
  tf_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}


template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

void
PlannerServer::computePlan()
{
  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();
  RCLCPP_INFO(get_logger(), "Computing path to goal.");

  geometry_msgs::msg::PoseStamped start;

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }
    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Check if planning service is available
    if (!planning_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "X500 Planning Service is not available!");
      result->error_msg = "Planning service not available";
      result->error_code = ActionToPoseResult::UNKNOWN;
      action_server_pose_->terminate_current(result);
      return;
    }

    // Prepare service request
    auto request = std::make_shared<x500_trajectory_planner::srv::X500PlanningService::Request>();
    request->target_pose = goal->goal.pose;
    request->planning_time = 10.0;  // TODO: Make this configurable
    request->planning_attempts = 3; // TODO: Make this configurable

    // Send async request with timeout
    auto future = planning_client_->async_send_request(request);
    
    // Wait for response with cancellation check
    auto timeout = std::chrono::seconds(15);
    auto start_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < timeout) {
      // Check for cancellation during planning
      if (action_server_pose_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Planning cancelled by user request");
        result->error_msg = "Goal was canceled during planning";
        action_server_pose_->terminate_all();
        return;
      }

      // Check if future is ready
      if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        break;
      }
    }

    // Check if we timed out
    if (std::chrono::steady_clock::now() - start_time >= timeout) {
      RCLCPP_ERROR(get_logger(), "Planning service request timed out");
      result->error_msg = "Planning service timeout";
      result->error_code = ActionToPoseResult::TIMEOUT;
      action_server_pose_->terminate_current(result);
      return;
    }

    // Get the response
    auto response = future.get();
    
    if (response->success) {
      // Convert trajectory to nav_msgs::Path
      result->path = convertTrajectoryToPath(response->trajectory_poses, goal->goal.header.frame_id);
      
      RCLCPP_INFO(get_logger(), "Planning successful! Generated path with %zu waypoints, distance: %.2f m",
                  result->path.poses.size(), response->linear_distance);

      // Publish plan for visualization
      publishPlan(result->path);
      
      action_server_pose_->succeeded_current(result);
      
    } else {
      RCLCPP_ERROR(get_logger(), "Planning failed: %s", response->message.c_str());
      result->error_msg = response->message;
      
      // Map planning failure to appropriate error code
      if (response->message.find("timeout") != std::string::npos) {
        result->error_code = ActionToPoseResult::TIMEOUT;
      } else if (response->message.find("no valid path") != std::string::npos) {
        result->error_code = ActionToPoseResult::NO_VALID_PATH;
      } else {
        result->error_code = ActionToPoseResult::UNKNOWN;
      }
      
      action_server_pose_->terminate_current(result);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Exception during planning: %s", ex.what());
    result->error_msg = std::string("Planning exception: ") + ex.what();
    result->error_code = ActionToPoseResult::UNKNOWN;
    action_server_pose_->terminate_current(result);
  }
} 

// TODO: Sviluppo planning attraverso waypoints
// void PlannerServer::computePlanThroughPoses()
// {
//
//   // Initialize the ComputePathThroughPoses goal and result
//   auto goal = action_server_poses_->get_current_goal();
//   auto result = std::make_shared<ActionThroughPoses::Result>();
//   nav_msgs::msg::Path concat_path;
//   RCLCPP_INFO(get_logger(), "Computing path through poses to goal.");
//
//   geometry_msgs::msg::PoseStamped curr_start, curr_goal;
//
//   try {
//     if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
//       return;
//     }
//
//     waitForCostmap();
//
//     getPreemptedGoalIfRequested(action_server_poses_, goal);
//
//     if (goal->goals.goals.empty()) {
//       throw nav2_core::NoViapointsGiven("No viapoints given");
//     }
//
//     // Use start pose if provided otherwise use current robot pose
//     geometry_msgs::msg::PoseStamped start;
//     if (!getStartPose<ActionThroughPoses>(goal, start)) {
//       throw nav2_core::PlannerTFError("Unable to get start pose");
//     }
//
//     auto cancel_checker = [this]() {
//         return action_server_poses_->is_cancel_requested();
//       };
//
//     // Get consecutive paths through these points
//     for (unsigned int i = 0; i != goal->goals.goals.size(); i++) {
//       // Get starting point
//       if (i == 0) {
//         curr_start = start;
//       } else {
//         // pick the end of the last planning task as the start for the next one
//         // to allow for path tolerance deviations
//         curr_start = concat_path.poses.back();
//         curr_start.header = concat_path.header;
//       }
//       curr_goal = goal->goals.goals[i];
//
//       // Transform them into the global frame
//       if (!transformPosesToGlobalFrame(curr_start, curr_goal)) {
//         throw nav2_core::PlannerTFError("Unable to transform poses to global frame");
//       }
//
//       // Get plan from start -> goal
//       nav_msgs::msg::Path curr_path = getPlan(
//         curr_start, curr_goal, goal->planner_id,
//         cancel_checker);
//
//       // Concatenate paths together
//       concat_path.poses.insert(
//         concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
//       concat_path.header = curr_path.header;
//     }
//
//     // Publish the plan for visualization purposes
//     result->path = concat_path;
//     publishPlan(result->path);
//
//     action_server_poses_->succeeded_current(result);
//   } catch (nav2_core::InvalidPlanner & ex) {
//     exceptionWarning(curr_start, curr_goal, goal->planner_id, ex, result->error_msg);
//     result->error_code = ActionThroughPosesResult::INVALID_PLANNER;
//     action_server_poses_->terminate_current(result);
//   } catch (nav2_core::StartOccupied & ex) {
// }

nav_msgs::msg::Path 
PlannerServer::convertTrajectoryToPath(
  const std::vector<geometry_msgs::msg::PoseStamped>& trajectory_poses,
  const std::string& frame_id)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = get_clock()->now();
  
  path.poses.reserve(trajectory_poses.size());
  
  for (const auto& pose_stamped : trajectory_poses) {
    geometry_msgs::msg::PoseStamped path_pose = pose_stamped;
    path_pose.header.frame_id = frame_id;
    path.poses.push_back(path_pose);
  }
  
  RCLCPP_DEBUG(get_logger(), "Converted trajectory to path with %zu poses", path.poses.size());
  return path;
}

void PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    auto msg = std::make_unique<nav_msgs::msg::Path>(path);
    plan_publisher_->publish(std::move(msg));

    // Poses log
    RCLCPP_DEBUG(get_logger(), "📊 Published path with %zu poses:", path.poses.size());
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto& pose_stamped = path.poses[i];
        const auto& pos = pose_stamped.pose.position;
        const auto& orient = pose_stamped.pose.orientation;

        tf2::Quaternion tf_q(orient.x, orient.y, orient.z, orient.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        
        RCLCPP_DEBUG(get_logger(), 
            "   [%3zu] pos=[%7.3f, %7.3f, %7.3f] "
            "yaw=%6.1f°",
            i,
            pos.x, pos.y, pos.z,                           // Position XYZ
            yaw * 180.0 / M_PI                             // Yaw in degrees
        );
    }

    RCLCPP_DEBUG(get_logger(), "Published path with %zu poses for visualization", path.poses.size());
  }
}

// void PlannerServer::isPathValid(
//   const std::shared_ptr<rmw_request_id_t>/*request_header*/,
//   const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
//   std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
// {
//   response->is_valid = true;

//   if (request->path.poses.empty()) {
//     response->is_valid = false;
//     return;
//   }

//   geometry_msgs::msg::PoseStamped current_pose;
//   unsigned int closest_point_index = 0;
//   if (costmap_ros_->getRobotPose(current_pose)) {
//     float current_distance = std::numeric_limits<float>::max();
//     float closest_distance = current_distance;
//     geometry_msgs::msg::Point current_point = current_pose.pose.position;
//     for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
//       geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

//       current_distance = nav2_util::geometry_utils::euclidean_distance(
//         current_point,
//         path_point);

//       if (current_distance < closest_distance) {
//         closest_point_index = i;
//         closest_distance = current_distance;
//       }
//     }

//     /**
//      * The lethal check starts at the closest point to avoid points that have already been passed
//      * and may have become occupied. The method for collision detection is based on the shape of
//      * the footprint.
//      */
//     std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
//     unsigned int mx = 0;
//     unsigned int my = 0;

//     bool use_radius = costmap_ros_->getUseRadius();

//     unsigned int cost = nav2_costmap_2d::FREE_SPACE;
//     for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
//       auto & position = request->path.poses[i].pose.position;
//       if (use_radius) {
//         if (costmap_->worldToMap(position.x, position.y, mx, my)) {
//           cost = costmap_->getCost(mx, my);
//         } else {
//           cost = nav2_costmap_2d::LETHAL_OBSTACLE;
//         }
//       } else {
//         nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
//         auto theta = tf2::getYaw(request->path.poses[i].pose.orientation);
//         cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
//             position.x, position.y, theta, footprint));
//       }

//       if (cost == nav2_costmap_2d::NO_INFORMATION && request->consider_unknown_as_obstacle) {
//         cost = nav2_costmap_2d::LETHAL_OBSTACLE;
//       } else if (cost == nav2_costmap_2d::NO_INFORMATION) {
//         cost = nav2_costmap_2d::FREE_SPACE;
//       }

//       if (use_radius &&
//         (cost >= request->max_cost || cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
//         cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
//       {
//         response->is_valid = false;
//         break;
//       } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost >= request->max_cost) {
//         response->is_valid = false;
//         break;
//       }
//     }
//   }
// }

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
