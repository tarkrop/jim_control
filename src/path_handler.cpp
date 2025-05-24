#include "jim_control/path_handler.hpp"

void PathHandler::set_path_length(const double local_path_length) { max_distance_ = local_path_length;}

double PathHandler::calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

int PathHandler::find_closest_point_index(
  const nav_msgs::msg::Path& global_path,
  const geometry_msgs::msg::PoseStamped& current_pose)
{
  if (global_path.poses.empty()) {
      return -1;
  }

  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_index = 0;
  for (size_t i = 0; i < global_path.poses.size(); ++i) {
      double dx = global_path.poses[i].pose.position.x - current_pose.pose.position.x;
      double dy = global_path.poses[i].pose.position.y - current_pose.pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          closest_index = static_cast<int>(i);
      }
  }

  return closest_index;
}

nav_msgs::msg::Path PathHandler::extract_path_segment(
  const nav_msgs::msg::Path& global_path,
  int start_index)
{
  nav_msgs::msg::Path path_segment;

  if (global_path.poses.empty() || start_index < 0 || start_index >= global_path.poses.size()) {
    return path_segment;
  }

  double accumulated_length = 0.0;
  path_segment.poses.push_back(global_path.poses[start_index]);

  for (size_t i = start_index + 1; i < global_path.poses.size(); ++i) {
    double segment_distance = calculate_distance(
      global_path.poses[i-1].pose.position,
      global_path.poses[i].pose.position
    );

    if (accumulated_length + segment_distance > max_distance_) {
      break;
    }

    accumulated_length += segment_distance;
    path_segment.poses.push_back(global_path.poses[i]);
  }

  return path_segment;
}