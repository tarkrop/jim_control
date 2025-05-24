#ifndef CONTROLLER_SERVER_HPP_
#define CONTROLLER_SERVER_HPP_

extern "C" {
  #include "acado_common.h"
  #include "acado_auxiliary_functions.h"
}
  
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// #include "Eigen3/Eigen/Core"
// #include "Eigen3/Eigen/QR"

#include <tf2/utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "jim_control/goal_checker.hpp"
#include "jim_control/path_handler.hpp"
// #include "jim_control/acado_mpc.hpp"

#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */


#define Ts 0.1 // sampling time
#define Lf 1.0
#define WP_NUM 1
#define MAX_VEL 0.3

#define OBS_COUNT 5
#define OBS_R 0.8
#define OBS_SOFT 1
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

using namespace std::chrono_literals;

class ControllerServer : public rclcpp::Node
{
public:
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ControllerServer();

private:
  
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  std::vector<std::vector<double>> init_acado();
  void init_weight(
    double weight_x, double weight_y, 
    double weight_q, double weight_v,
    double weight_w, double weight_a,
    double weight_j, double weight_o);

  std::vector<std::vector<double>> run_mpc_acado(
    std::vector<double> states, 
    std::vector<double> ref_states,
    std::vector<std::vector<double>> previous_u
  );

  std::vector<double> motion_prediction
  (
    const std::vector<double> &cur_states,                             
    const std::vector<std::vector<double>> &prev_a
  );

  std::vector<double> correct_angle_refference
  (
    std::vector<double> predicted_states,
    std::vector<double> ptsq
  );

  std::vector<double> calculate_ref_states
  (
    const std::vector<double> &ref_x,
    const std::vector<double> &ref_y,
    const std::vector<double> &ref_q,
    const double &reference_a,
    const double &reference_alpha
  );

  std::vector<double> update_states
  (
    std::vector<double> state, 
    double a_cmd,
    double alpha_cmd
  );

  void update_obstacles();
  
  float quaternion_to_yaw(geometry_msgs::msg::Quaternion orientation);
  geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

  double calculate_distance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2);

  void current_pose_callback();
  void local_map_callback();

  void odom_callback();
  void obstacle_callback();

  std::unique_ptr<GoalChecker> goal_checker_;
  std::unique_ptr<PathHandler> path_handler_;

  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cur_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obs_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr state_pub_;

  std::thread execute_thread_;

  rclcpp::Clock clock;
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path local_path_;
  geometry_msgs::msg::PoseStamped cur_pos_;
  geometry_msgs::msg::Twist cur_vel_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  std::vector<geometry_msgs::msg::Point> obs_points_;


  std::shared_ptr<FollowPath::Goal> goal_;

  
  std::shared_ptr<GoalHandleFollowPath> current_follow_handle_;
  std::mutex follow_mutex_;

  std::vector<std::vector<double>> control_output_;
};

#endif