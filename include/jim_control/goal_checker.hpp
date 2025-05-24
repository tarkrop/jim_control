#include <memory>
#include <string>
#include <vector>
#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/utils.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GoalChecker
{
public:
  GoalChecker() {};

  ~GoalChecker() {}

  void initialize(double xy_goal_tolerance,
    double yaw_goal_tolerance, double xy_goal_tolerance_sq);
  void reset_check();

  double goal_distance(
    const geometry_msgs::msg::Pose & query_pose, 
    const geometry_msgs::msg::Pose & goal_pose);
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, 
    const geometry_msgs::msg::Pose & goal_pose);

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance
  );

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  double xy_goal_tolerance_sq_;

};