#include "jim_control/controller_server.hpp"

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: Node("controller_server", options)
{
  control_output_ = init_acado();

  goal_checker_ = std::make_unique<GoalChecker>();
  path_handler_ = std::make_unique<PathHandler>();
  goal_checker_->initialize(0.05, 0.8, 0.1);
  path_handler_->set_path_length(5); // MPC에 전달하는 경로 길이 5m

  vel_pub_= this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel1", 1);
  path_pub_= this->create_publisher<nav_msgs::msg::Path>("predict_path", 1);
  state_pub_= this->create_publisher<nav_msgs::msg::Path>("local_path", 1);

  this->action_server_ = rclcpp_action::create_server<FollowPath>(
    this,
    "follow_path",
    std::bind(&ControllerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ControllerServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ControllerServer::handle_accepted, this, std::placeholders::_1));

  current_pose_callback();
  local_map_callback();
  odom_callback();
  obstacle_callback();

  current_follow_handle_ = nullptr;

  RCLCPP_INFO(this->get_logger(), "FollowPath Action Server has been started.");
}

ControllerServer::~ControllerServer()
{
  std::lock_guard<std::mutex> lock(follow_mutex_);
  if (current_follow_handle_) {
    RCLCPP_WARN(this->get_logger(), "Node shutting down. Aborting current goal.");
    current_follow_handle_->abort(nullptr);
    current_follow_handle_ = nullptr;
  }

  if (execute_thread_.joinable()) {
    execute_thread_.join();
  }
}

rclcpp_action::GoalResponse ControllerServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowPath::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request with %zu poses in path.", goal->path.poses.size());
  if (goal->path.poses.empty()) {return rclcpp_action::GoalResponse::REJECT;}
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse ControllerServer::handle_cancel(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request for goal UUID %s",
              rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ControllerServer::handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted. Starting execution thread.");

  if (current_follow_handle_)
  {
    auto result = std::make_shared<FollowPath::Result>();
    current_follow_handle_->abort(result);
    RCLCPP_INFO(this->get_logger(), "Aborting previous goal");
  }

  if (execute_thread_.joinable()) {
    execute_thread_.join();
  }

  current_follow_handle_ = goal_handle;
  current_follow_handle_->execute();
  execute_thread_ = std::thread(std::bind(&ControllerServer::execute, this, current_follow_handle_));
}

void ControllerServer::execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "==================================");
  RCLCPP_INFO(this->get_logger(), "Executing goal...");

  auto goal = goal_handle->get_goal()->path;
  auto path = goal;

  auto result = std::make_shared<FollowPath::Result>();

  auto feedback = std::make_shared<FollowPath::Feedback>();

  rclcpp::Rate loop_rate(20);
  goal_checker_->reset_check();

  while(rclcpp::ok())
  {
    geometry_msgs::msg::PoseStamped current_robot_pose;
    current_robot_pose = cur_pos_;

    if (!current_follow_handle_->is_executing())
    {
      RCLCPP_INFO(this->get_logger(), "Abort Complete");
      break;
    }
    else if (goal_checker_->isGoalReached(current_robot_pose.pose, path.poses[path.poses.size()-1].pose)) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      geometry_msgs::msg::Twist vel;
      vel.linear.x = 0;
      vel.angular.z = 0;
      vel_pub_->publish(vel);
      goal_handle->succeed(result);
      current_follow_handle_ = nullptr;
      break;
    }

    auto closest_index = path_handler_->find_closest_point_index(path, current_robot_pose);
    auto local_path = path_handler_->extract_path_segment(path, closest_index);
    
    double local_path_length = MAX_VEL * Ts * ACADO_N;
    nav_msgs::msg::Path trimmed_local_path;
    double cumulative_distance = 0.0;
    trimmed_local_path.poses.push_back(local_path.poses[0]);
    for (size_t i = 1; i < local_path.poses.size(); ++i) {
      double segment_distance = calculate_distance(local_path.poses[i-1], local_path.poses[i]);
      cumulative_distance += segment_distance;
      trimmed_local_path.poses.push_back(local_path.poses[i]);

      if (cumulative_distance >= local_path_length) {
          break;
      }
    }
    local_path_ = trimmed_local_path;

    double px = current_robot_pose.pose.position.x;
    double py = current_robot_pose.pose.position.y;
    double pq = quaternion_to_yaw(current_robot_pose.pose.orientation);
    std::vector<double> cur_state = {px, py, pq, cur_vel_.linear.x, cur_vel_.angular.z};
    std::vector<double> ptsx, ptsy, ptsq;

    for (int i = 0; i < ACADO_N; i++)
    {
      double pred_x, pred_y, pred_q;
      if (WP_NUM * i >= local_path_.poses.size())
      {
        pred_x = local_path_.poses[local_path_.poses.size()-1].pose.position.x;
				pred_y = local_path_.poses[local_path_.poses.size()-1].pose.position.y;
				pred_q = quaternion_to_yaw(local_path_.poses[local_path_.poses.size()-1].pose.orientation);
				ptsx.push_back(pred_x);
				ptsy.push_back(pred_y);
				ptsq.push_back(pred_q);
      }
      else
      {
        pred_x = local_path_.poses[WP_NUM * i].pose.position.x;
				pred_y = local_path_.poses[WP_NUM * i].pose.position.y;
				pred_q = quaternion_to_yaw(local_path_.poses[WP_NUM * i].pose.orientation);
				ptsx.push_back(pred_x);
				ptsy.push_back(pred_y);
				ptsq.push_back(pred_q);
      }
    }

    double reference_a = 0.0;
		double reference_alpha = 0.0;
    // control_output_[0][0] = cur_vel_.linear.x;
    // control_output_[1][0] = cur_vel_.angular.z;

    update_obstacles();
    for (int i = 0; i < N; i++)
    {
    acadoVariables.lbAValues[i*2+0] = 0;
    }
    std::vector<double> predicted_states = motion_prediction(cur_state, control_output_);
    // auto ptsq_debug = ptsq;
    // ptsq = correct_angle_refference(predicted_states, ptsq);
    std::vector<double> ref_states = calculate_ref_states(ptsx, ptsy, ptsq, reference_a, reference_alpha);
    control_output_ = run_mpc_acado(predicted_states, ref_states, control_output_);

    nav_msgs::msg::Path predict_path;
    predict_path.header.frame_id = "map";
	  predict_path.header.stamp = clock.now();
    for (int i = 0; i < ACADO_N; i++)
    {
      geometry_msgs::msg::PoseStamped pred_pose;
      pred_pose.header.stamp = clock.now();
      pred_pose.pose.position.x = acadoVariables.x[NX * i + 0];
      pred_pose.pose.position.y = acadoVariables.x[NX * i + 1];
      pred_pose.pose.orientation = yaw_to_quaternion(acadoVariables.x[NX * i + 2]);
      predict_path.poses.push_back(pred_pose);
    }

    nav_msgs::msg::Path l_path;
    l_path.header.frame_id = "map";
	  l_path.header.stamp = clock.now();
    for (int i = 0; i < ptsx.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pred_pose;
      pred_pose.header.stamp = clock.now();
      pred_pose.pose.position.x = ptsx[i];
      pred_pose.pose.position.y = ptsy[i];
      pred_pose.pose.orientation = yaw_to_quaternion(ptsq[i]);
      l_path.poses.push_back(pred_pose);
    }

    // std::cout << local_path_length << std::endl;

    // for (int i = 0; i < N; i++)
    // {
    //   std::cout << "Path " << i << " "
    //       << ptsx[i] << " "
    //       << ptsy[i] << " "
    //       << ptsq[i] << std::endl;
    // }


    // std::cout << "==============================" << std::endl;
    // for (int i = 0; i < N; i++)
    // {
    //   std::cout << "State " << i << " "
    //       << acadoVariables.x[NX * i + 0] << " "
    //       << acadoVariables.x[NX * i + 1] << " "
    //       << acadoVariables.x[NX * i + 2] << " " 
    //       << acadoVariables.x[NX * i + 3] << " "
    //       << acadoVariables.x[NX * i + 4] <<std::endl;
    // }

    // for (int i = 0; i < N; i++)
    // {
    //   std::cout << "Control " << i << " "
    //       << acadoVariables.u[NU * i + 0] << " "
    //       << acadoVariables.u[NU * i + 1] << std::endl;
    // }

    geometry_msgs::msg::Twist vel;
    vel.linear.x = control_output_[0][0] * Ts + cur_vel_.linear.x;
		vel.angular.z = control_output_[1][0] * Ts + cur_vel_.angular.z;

    feedback->distance_to_goal = goal_checker_->goal_distance(current_robot_pose.pose, path.poses[path.poses.size()-1].pose);
    // std::cout << "Twist " << " "
    //       << vel.linear.x << " "
    //       << vel.angular.z << std::endl;
    vel_pub_->publish(vel);
    path_pub_->publish(predict_path);
    state_pub_->publish(l_path);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
}

void ControllerServer::current_pose_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  cur_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose",
    qos,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
    { 
      cur_pos_.header.stamp.sec = 0;
      cur_pos_.header.stamp.nanosec = 0;
      cur_pos_.header.frame_id = "map";
      cur_pos_.pose = msg->pose.pose;
      // RCLCPP_INFO(this->get_logger(), "%f, %f", cur_pos_.pose.position.x, cur_pos_.pose.position.y);

    }
  );
}

void ControllerServer::local_map_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap", qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      local_costmap_ = msg;
      RCLCPP_INFO(this->get_logger(), "Received OccupancyGrid with size: %d x %d",
                  local_costmap_->info.width, local_costmap_->info.height);
    });
}

void ControllerServer::odom_callback()
{ 
  cur_vel_.linear.x = 0;
  cur_vel_.angular.z = 0;
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      cur_vel_.linear.x = msg->twist.twist.linear.x;
      cur_vel_.angular.z = msg->twist.twist.angular.z;
    }
  );
}

void ControllerServer::obstacle_callback()
{

  obs_points_.reserve(OBS_COUNT);
  for (int i = 0; i < OBS_COUNT; i++)
  {
    geometry_msgs::msg::Point default_point;
    default_point.x = 1000;
    default_point.y = 1000;
    default_point.z = 0;
    obs_points_.push_back(default_point);
  }

  obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/mean_global_points", 1,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);
      std::vector<geometry_msgs::msg::Point> points;
      points.reserve(cloud.size());

      for (const auto& point: cloud.points)
      { 
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        points.push_back(p);
      }

      std::sort(points.begin(), points.end(), 
        [](const geometry_msgs::msg::Point& a, geometry_msgs::msg::Point& b)
        {
          return a.x * a.x + a.y * a.y < b.x * b.x + b.y * b.y; 
        }
      );

      std::vector<geometry_msgs::msg::Point> obs_points;

      if (points.size() < OBS_COUNT)
      {
        obs_points.assign(points.begin(), points.end());
        geometry_msgs::msg::Point default_point;
        default_point.x = 1000.0;
        default_point.y = 1000.0;
        default_point.z = 0.0;

        while (obs_points.size() < OBS_COUNT)
        {
          obs_points.push_back(default_point);
        }
      }
      else
      {
        obs_points.assign(points.begin(), points.begin()+5);
      }

      obs_points_ = obs_points;
    }
  );
}

float ControllerServer::quaternion_to_yaw(geometry_msgs::msg::Quaternion orientation)
{
double q0 = orientation.x;
double q1 = orientation.y;
double q2 = orientation.z;
double q3 = orientation.w;

float yaw = atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2));
return yaw;
};

geometry_msgs::msg::Quaternion ControllerServer::yaw_to_quaternion(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    double half_yaw = yaw / 2.0;
    q.w = std::cos(half_yaw);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(half_yaw);
    return q;
}

void ControllerServer::init_weight(
  double weight_x, double weight_y, double weight_q, double weight_v, 
  double weight_w, double weight_a, double weight_j, double weight_o)
{
	for (int i = 0; i < N; i++)
	{
		// Setup diagonal entries
		acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_x;
		acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_y;
		acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_q;
		acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_v;
		acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_w;
		acadoVariables.W[NY * NY * i + (NY + 1) * 5] = weight_a;
		acadoVariables.W[NY * NY * i + (NY + 1) * 6] = weight_j;
		acadoVariables.W[NY * NY * i + (NY + 1) * 7] = weight_o;
	}
	acadoVariables.WN[(NYN + 1) * 0] = weight_x * 10;
	acadoVariables.WN[(NYN + 1) * 1] = weight_y * 10;
	acadoVariables.WN[(NYN + 1) * 2] = weight_q * 10;
}

std::vector<std::vector<double>> ControllerServer::init_acado()
{
    /* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (int i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.00001;

	/* Initialize the measurements/reference. */
	for (int i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (int i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

  acado_preparationStep();
	std::vector<double> control_output_a;
  std::vector<double> control_output_j;
	for (int i = 0; i < ACADO_N; ++i)
	{
    // There are 3 outputs vx, vy, w
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_a.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
			else 
			{
				control_output_j.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
  // init_weight(100.0, 100.0, 0.5, 1.0, 1.0, 0.0, 0.0, 0.0);
  init_weight(100, 100, 0.5, 10.0, 10.0, 5.0, 0.5, 5.0);
  // init_weight(100, 100, 0.5, 10.0, 10.0, 10.0, 0.5, 1.0);

  for (int i = 0; i < (N + 1) * NOD; ++i)
  {
    acadoVariables.od[i] = 0.0;
  }
    
	return {control_output_a, control_output_j};
}

std::vector<std::vector<double>> ControllerServer::run_mpc_acado(
  std::vector<double> states, 
  std::vector<double> ref_states,
  std::vector<std::vector<double>> previous_u)
{
  /* Some temporary variables. */
  int i, iter;
  acado_timer t;

  /* Initialize the states and controls. */
  for (i = 0; i < NX * (N + 1); ++i)  
  {
  acadoVariables.x[ i ] = (real_t) states[i];
  }
  for (i = 0; i < NX; ++i)
  {
  acadoVariables.x0[i] = (real_t)states[i];
  }

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i)
  {
  acadoVariables.y[i] = (real_t)ref_states[i];
  }
  for (i = 0; i < NYN; ++i)
  {
  acadoVariables.yN[i] = (real_t)ref_states[NY * (N - 1) + i];
  }

  
  // /* Prepare first step */
  acado_preparationStep();

  /* Get the time before start of the loop. */
  acado_tic(&t);

  /* The "real-time iterations" loop. */

  for (iter = 0; iter < NUM_STEPS; ++iter)
  {
  /* Perform the feedback step. */
  acado_feedbackStep();
  acado_preparationStep();

  /* Optional: shift the initialization (look at acado_common.h). */
  // acado_shiftStates(2, 0, 0);
  // acado_shiftControls( 0 );
  }

  std::cout << "========================================" << std::endl;
  // Reference
  // for (int i = 0; i < 5; i++)
  // {
  // 	std::cout << "Reference " << i << " "
  // 			<< acadoVariables.y[NY*i + 0] << " "
  // 			<< acadoVariables.y[NY*i + 1] << " "
  // 			<< acadoVariables.y[NY*i + 2] << " "
  // 			<< acadoVariables.y[NY*i + 3] << " "
  // 			<< acadoVariables.y[NY*i + 4] << " "
  // 			<< acadoVariables.y[NY*i + 5] << " "
  // 			<< acadoVariables.y[NY*i + 6] << " "
  // 			<< acadoVariables.y[NY*i + 7] << std::endl;
  // }

  // std::cout << "Last Reference " << " "
  // 			<< acadoVariables.yN[0] << " "
  // 			<< acadoVariables.yN[1] << " "
  // 			<< acadoVariables.yN[2] << std::endl;

  // for (int i = 0; i < 5; i++)
  // {
  // 	std::cout << "Control " << i << " "
  // 			<< acadoVariables.u[NU * i + 0] << " "
  // 			<< acadoVariables.u[NU * i + 1] <<  std::endl;
  // }
  // // state:

  //   std::cout << "First State " << " "
  // 			      << acadoVariables.x0[0] << " "
  // 			      << acadoVariables.x0[1] << " "
  // 			      << acadoVariables.x0[2] << " "
  //             << acadoVariables.x0[3] << " "
  //             << acadoVariables.x0[4] << std::endl;
  // for (int i = 0; i < 5; i++)
  // {
  //   std::cout << "State " << i << " "
  // 			      << acadoVariables.x[NX * i + 0] << " "
  // 			      << acadoVariables.x[NX * i + 1] << " "
  // 			      << acadoVariables.x[NX * i + 2] << " "
  //             << acadoVariables.x[NX * i + 3] << " "
  //             << acadoVariables.x[NX * i + 4] << std::endl;
  // }
  std::cout << "Real-Time Iteration = " << iter << ", KKT Tolerance = " << acado_getKKT() << std::endl;
  // for (int i = 0; i < 15; ++i)
  // {
  //   std::cout << acadoVariables.od[i] << std::endl;
  // }

  // double eps = 1e-7;
  // double obs_total_cost = 0.0;

  // for (int i = 0; i < N; i++)
  // {
  // //   auto obs_cost =
  // //   1 / (1.0 + exp(-1 + sqrt((pow(acadoVariables.x[NX * i + 0]-acadoVariables.od[NOD + 0], 2) + pow(acadoVariables.x[NX * i + 1]-acadoVariables.od[NOD + 1], 2))/pow(acadoVariables.od[NOD + 2]+eps,2)))) +
  // //   1 / (1.0 + exp(-1 + sqrt((pow(acadoVariables.x[NX * i + 0]-acadoVariables.od[NOD + 3], 2) + pow(acadoVariables.x[NX * i + 1]-acadoVariables.od[NOD + 4], 2))/pow(acadoVariables.od[NOD + 5]+eps,2)))) +
  // //   1 / (1.0 + exp(-1 + sqrt((pow(acadoVariables.x[NX * i + 0]-acadoVariables.od[NOD + 6], 2) + pow(acadoVariables.x[NX * i + 1]-acadoVariables.od[NOD + 7], 2))/pow(acadoVariables.od[NOD + 8]+eps,2)))) +
  // //   1 / (1.0 + exp(-1 + sqrt((pow(acadoVariables.x[NX * i + 0]-acadoVariables.od[NOD + 9], 2) + pow(acadoVariables.x[NX * i + 1]-acadoVariables.od[NOD + 10], 2))/pow(acadoVariables.od[NOD + 11]+eps,2)))) +
  // //   1 / (1.0 + exp(-1 + sqrt((pow(acadoVariables.x[NX * i + 0]-acadoVariables.od[NOD + 12], 2) + pow(acadoVariables.x[NX * i + 1]-acadoVariables.od[NOD + 13], 2))/pow(acadoVariables.od[NOD + 14]+eps,2))));

  //   auto obs_cost = sqrt(pow(acadoVariables.x[NX * i + 0]-acadoVariables.od[NOD + 0], 2) + pow(acadoVariables.x[NX * i + 1]-acadoVariables.od[NOD + 1], 2)) - (acadoVariables.od[NOD + 2] + eps);
  //   obs_total_cost += 1/obs_cost;

  // }
  


  /* Read the elapsed time. */
  real_t te = acado_toc(&t);

  std::vector<double> control_output_a;
  std::vector<double> control_output_j;
  real_t *u = acado_getVariablesU();

  for (int i = 0; i < ACADO_N; ++i)
  {
  for (int j = 0; j < ACADO_NU; ++j)
  {
  if (j == 0)
  {
  control_output_a.push_back((double)u[i * ACADO_NU + j]);
  }
  else
  {
  control_output_j.push_back((double)u[i * ACADO_NU + j]);
  }
  }
  }
  return {control_output_a, control_output_j};
}

std::vector<double> ControllerServer::calculate_ref_states(
  const std::vector<double> &ref_x,
  const std::vector<double> &ref_y,
  const std::vector<double> &ref_q,
  const double &reference_a,
  const double &reference_alpha)
{
  std::vector<double> result;
  for (int i = 0; i < N; i++)
  {
  result.push_back(ref_x[i]);
  result.push_back(ref_y[i]);
  result.push_back(ref_q[i]);
  result.push_back(MAX_VEL); // ref_v
  result.push_back(0); // ref_w
  result.push_back(0); // ref_a
  result.push_back(0); // ref_j
  result.push_back(0); // ref_obs
  }
  return result;
}

std::vector<double> ControllerServer::update_states(
  std::vector<double> state,
  double a_cmd, double alpha_cmd)
{
	double x0 = state[0];
	double y0 = state[1];
	double q0 = state[2];
	double v0 = state[3];
	double w0 = state[4];
  double a0 = a_cmd;
  double alpha0 = alpha_cmd;

	double x1 = x0 + (v0 * cos(q0))* Ts;
	double y1 = y0 + (v0 * sin(q0))* Ts;
	double q1 = q0 + w0 * Ts;
  double v1 = v0 + a0 * Ts;
  double w1 = w0 + alpha0 * Ts;

	return {x1, y1, q1, v1, w1};
}

void ControllerServer::update_obstacles()
{
  for (int i = 0; i < (N + 1); ++i)
  {
    acadoVariables.od[i * NOD + 0] = (real_t)obs_points_[0].x;
    acadoVariables.od[i * NOD + 1] = (real_t)obs_points_[0].y;
    acadoVariables.od[i * NOD + 2] = (real_t)OBS_R;
    acadoVariables.od[i * NOD + 3] = (real_t)obs_points_[1].x;
    acadoVariables.od[i * NOD + 4] = (real_t)obs_points_[1].y;
    acadoVariables.od[i * NOD + 5] = (real_t)OBS_R;
    acadoVariables.od[i * NOD + 6] = (real_t)obs_points_[2].x;
    acadoVariables.od[i * NOD + 7] = (real_t)obs_points_[2].y;
    acadoVariables.od[i * NOD + 8] = (real_t)OBS_R;
    acadoVariables.od[i * NOD + 9] = (real_t)obs_points_[3].x;
    acadoVariables.od[i * NOD + 10] = (real_t)obs_points_[3].y;
    acadoVariables.od[i * NOD + 11] = (real_t)OBS_R;
    acadoVariables.od[i * NOD + 12] = (real_t)obs_points_[4].x;
    acadoVariables.od[i * NOD + 13] = (real_t)obs_points_[4].y;
    acadoVariables.od[i * NOD + 14] = (real_t)OBS_R;
  }
}

std::vector<double> ControllerServer::motion_prediction(const std::vector<double> &cur_states,
  const std::vector<std::vector<double>> &prev_a)
{
  std::vector<double> old_a_cmd = prev_a[0];
	std::vector<double> old_alpha_cmd = prev_a[1];

	std::vector<std::vector<double>> predicted_states;
	predicted_states.push_back(cur_states);

	for (int i = 0; i < N; i++)
	{
		std::vector<double> cur_state = predicted_states[i];
		// yaw angle compensation of overflow
		if (cur_state[3] > M_PI)
		{
			cur_state[3] -= 2 * M_PI;
		}
		if (cur_state[3] < -M_PI)
		{
			cur_state[3] += 2 * M_PI;
		}
		std::vector<double> next_state = update_states(cur_state, old_a_cmd[i], old_alpha_cmd[i]);
		predicted_states.push_back(next_state);
	}

	std::vector<double> result;
	for (int i = 0; i < (ACADO_N + 1); ++i)
	{
		for (int j = 0; j < NX; ++j)
		{
			result.push_back(predicted_states[i][j]);
		}
	}
	return result;
}

std::vector<double> ControllerServer::correct_angle_refference(std::vector<double> predicted_states, std::vector<double> ptsq)
{
	std::vector<double> predq;
	for (int i = 0; i < (ACADO_N + 1); ++i)
	{
		predq.push_back(predicted_states[i * ACADO_NX + 2]);
	}
	for (int i = 0; i < (ACADO_N+1); ++i)
	{
		double errq = ptsq[i] - predq[i];
		if (errq > M_PI) {errq -= 2*M_PI;}
		else if (errq < -M_PI) {errq += 2*M_PI;}
		ptsq[i] = predq[i] + errq;
	}
	return ptsq;
}

double ControllerServer::calculate_distance(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2) {
  double dx = pose2.pose.position.x - pose1.pose.position.x;
  double dy = pose2.pose.position.y - pose1.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server_node = std::make_shared<ControllerServer>();
  rclcpp::spin(action_server_node);

  rclcpp::shutdown();

  return 0;
}
