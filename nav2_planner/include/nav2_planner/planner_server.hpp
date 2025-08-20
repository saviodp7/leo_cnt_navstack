#ifndef NAV2_PLANNER__PLANNER_SERVER_HPP_
#define NAV2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/service_server.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_core/planner_exceptions.hpp"

#include "x500_trajectory_planner/srv/x500_planning_service.hpp"

namespace nav2_planner
{
/**
 * @class nav2_planner::PlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class PlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_planner::PlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for nav2_planner::PlannerServer
   */
  ~PlannerServer() = default;

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionToPoseResult = ActionToPose::Result;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;
  // using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  // using ActionThroughPosesResult = ActionThroughPoses::Result;
  // using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param action_server Action server to terminate if required
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  // template<typename T>
  // bool getStartPose(
  //   typename std::shared_ptr<const typename T::Goal> goal,
  //   geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathToPose
   */
  void computePlan();

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathThroughPoses
   */
  // void computePlanThroughPoses();

  /**
   * @brief The service callback to determine if the path is still valid
   * @param request to the service
   * @param response from the service
   */
  // void isPathValid(
  //   const std::shared_ptr<rmw_request_id_t> request_header,
  //   const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  //   std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Convert trajectory poses to nav_msgs::Path
   * @param trajectory_poses Array of trajectory poses from MoveIt2
   * @param frame_id Target frame ID for the path
   * @return nav_msgs::Path
   */
  nav_msgs::msg::Path convertTrajectoryToPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& trajectory_poses,
    const std::string& frame_id);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);


  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_pose_;
  // std::unique_ptr<ActionServerThroughPoses> action_server_poses_;

  // Trajectory planning service through MoveIt2
  rclcpp::Client<x500_trajectory_planner::srv::X500PlanningService>::SharedPtr planning_client_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

 
  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Service to determine if the path is valid
//   nav2_util::ServiceServer<nav2_msgs::srv::IsPathValid,
//     std::shared_ptr<nav2_util::LifecycleNode>>::SharedPtr is_path_valid_service_;
};

}  // namespace nav2_planner

#endif  // NAV2_PLANNER__PLANNER_SERVER_HPP_
