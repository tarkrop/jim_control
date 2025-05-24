#include "jim_control/goal_checker.hpp"

// GoalChecker::GoalChecker()
// : xy_goal_tolerance_(0.25),
// yaw_goal_tolerance_(0.25),
// check_xy_(true),
// stateful_(true),
// xy_goal_tolerance_sq_(0.0625)
// {
// }

void GoalChecker::initialize(double xy_goal_tolerance,
  double yaw_goal_tolerance, double xy_goal_tolerance_sq)
{
  xy_goal_tolerance_ = xy_goal_tolerance;
  yaw_goal_tolerance_ = yaw_goal_tolerance;
  xy_goal_tolerance_sq_ = xy_goal_tolerance_sq;
}

void GoalChecker::reset_check()
{
  check_xy_ = true;
}

bool GoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  return true;
  // double dyaw = angles::shortest_angular_distance(
  //   tf2::getYaw(query_pose.orientation),
  //   tf2::getYaw(goal_pose.orientation));
  // return fabs(dyaw) < yaw_goal_tolerance_;
}

double GoalChecker::goal_distance(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  double dx = query_pose.position.x - goal_pose.position.x;
  double dy = query_pose.position.y - goal_pose.position.y;
  return sqrt(dx * dx + dy * dy);
}

bool GoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}