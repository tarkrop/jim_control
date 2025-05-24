
extern "C" {
  #include "acado_common.h"
  #include "acado_auxiliary_functions.h"
  }
  
  
#include "Eigen3/Eigen/Core"
#include "Eigen3/Eigen/QR"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

#define Ts 0.1 // sampling time
#define Lf 1.0

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class AcadoMPC
{
public:
  explicit AcadoMPC();
  virtual ~AcadoMPC() {};

  std::vector<std::vector<double>> init_acado();
  void init_weight(
    double weight_x, double weight_y, 
    double weight_q, double weight_vx,
    double weight_vy, double weight_w);

  std::vector<std::vector<double>> run_mpc_acado(
    std::vector<double> states, 
    std::vector<double> ref_states,
    std::vector<std::vector<double>> previous_u
  );

  std::vector<double> motion_prediction
  (
    const std::vector<double> &cur_states,                             
    const std::vector<std::vector<double>> &prev_u
  );

  std::vector<double> calculate_ref_states
  (
    const std::vector<double> &ref_x,
    const std::vector<double> &ref_y,
    const std::vector<double> &ref_q,
    const double &reference_vx,
    const double &reference_vy,
    const double &reference_w
  );

  std::vector<double> update_states
  (
    std::vector<double> state, 
    double vx_cmd, 
    double vy_cmd, 
    double w_cmd
  );
  
  float quaternion_to_yaw(geometry_msgs::msg::Quaternion orientation)
  {
  double q0 = orientation.x;
  double q1 = orientation.y;
  double q2 = orientation.z;
  double q3 = orientation.w;

  float yaw = atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2));
  return yaw;
  };

};