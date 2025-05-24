#include <cmath>
#include <vector>
#include <limits>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathHandler
{
public:
  PathHandler() {};

  ~PathHandler() {};
  
  void set_path_length(double local_path_length);

  double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);

  int find_closest_point_index(
    const nav_msgs::msg::Path& global_path,
    const geometry_msgs::msg::PoseStamped& current_pose);


  nav_msgs::msg::Path extract_path_segment(
    const nav_msgs::msg::Path& global_path,
    int start_index);

  protected:

  double max_distance_;
};