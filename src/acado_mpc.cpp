#include "jim_control/acado_mpc.hpp"

AcadoMPC::AcadoMPC() {};


void AcadoMPC::init_weight(
  double weight_x, double weight_y, 
  double weight_q, double weight_vx,
  double weight_vy, double weight_w)
{
	for (int i = 0; i < N; i++)
	{
		// Setup diagonal entries
		acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_x;
		acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_y;
		acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_q;
		acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_vx;
    acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_vy;
		acadoVariables.W[NY * NY * i + (NY + 1) * 5] = weight_w;
	}
	acadoVariables.WN[(NYN + 1) * 0] = weight_x;
	acadoVariables.WN[(NYN + 1) * 1] = weight_y;
	acadoVariables.WN[(NYN + 1) * 2] = weight_q;
}

std::vector<std::vector<double>> AcadoMPC::init_acado()
{
    /* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (int i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (int i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (int i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

  acado_preparationStep();
	std::vector<double> control_output_vx;
	std::vector<double> control_output_vy;
  std::vector<double> control_output_w;
	for (int i = 0; i < ACADO_N; ++i)
	{
    // There are 3 outputs vx, vy, w
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_vx.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
      else if(j == 1 )
      {
        control_output_vy.push_back(acadoVariables.u[i * ACADO_NU + j]);
      }
			else 
			{
				control_output_w.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
	return {control_output_vx, control_output_vy, control_output_w};
}

std::vector<std::vector<double>> AcadoMPC::run_mpc_acado(
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

  for (iter = 0; i < NUM_STEPS; ++i)
  {
  /* Perform the feedback step. */
  acado_feedbackStep();
  acado_preparationStep();

  /* Optional: shift the initialization (look at acado_common.h). */
  /* acado_shiftStates(2, 0, 0); */
  /* acado_shiftControls( 0 ); */
  }


  // // Reference
  // for (int i = 0; i < N; i++)
  // {
  // 	cout << "Reference " << i << " "
  // 			<< acadoVariables.y[NY*i + 0] << " "
  // 			<< acadoVariables.y[NY*i + 1] << " "
  // 			<< acadoVariables.y[NY*i + 2] << " "
  // 			<< acadoVariables.y[NY*i + 3] << " "
  // 			<< acadoVariables.y[NY*i + 4] << " "
  // 			<< acadoVariables.y[NY*i + 5] << endl;
  // }

  // for (int i = 0; i < N; i++)
  // {
  // 	cout << "Control " << i << " "
  // 			<< acadoVariables.u[NU * i + 0] << " "
  // 			<< acadoVariables.u[NU * i + 1] << " "
  // 			<< acadoVariables.u[NU * i + 2] << endl;
  // }

  // // state:
  // for (int i = 0; i < N; i++)
  // {
  // 	cout << "State " << i << " "
  // 			<< acadoVariables.x[NX * i + 0] << " "
  // 			<< acadoVariables.x[NX * i + 1] << " "
  // 			<< acadoVariables.x[NX * i + 2] << endl;
  // }

  /* Read the elapsed time. */
  real_t te = acado_toc(&t);

  std::vector<double> control_output_vx;
  std::vector<double> control_output_vy;
  std::vector<double> control_output_w;
  real_t *u = acado_getVariablesU();

  for (int i = 0; i < ACADO_N; ++i)
  {
  for (int j = 0; j < ACADO_NU; ++j)
  {
  if (j == 0)
  {
  control_output_vx.push_back((double)u[i * ACADO_NU + j]);
  }
  else if (j == 1)
  {
  control_output_vy.push_back((double)u[i * ACADO_NU + j]);
  }
  else
  {
  control_output_w.push_back((double)u[i * ACADO_NU + j]);
  }
  }
  }
  // cout << control_output_vx[0] << " " << control_output_vy[0] << " " << control_output_w[0] << endl;
  return {control_output_vx, control_output_vy, control_output_w};
}

std::vector<double> AcadoMPC::calculate_ref_states(
  const std::vector<double> &ref_x,
  const std::vector<double> &ref_y,
  const std::vector<double> &ref_q,
  const double &reference_vx,
  const double &reference_vy,
  const double &reference_w)
{
  std::vector<double> result;
  for (int i = 0; i < N; i++)
  {
  result.push_back(ref_x[i]);
  result.push_back(ref_y[i]);
  result.push_back(ref_q[i]);
  result.push_back(0);
  result.push_back(0);
  result.push_back(0);
  }
  return result;
}

std::vector<double> AcadoMPC::update_states(
  std::vector<double> state,
  double vx_cmd, double vy_cmd, double w_cmd)
{
	// based on kinematic model
	double x0 = state[0];
	double y0 = state[1];
	double q0 = state[2];
	double vx0 = vx_cmd;
	double vy0 = vy_cmd;
	double w0 = w_cmd;

	double x1 = x0 + (vx0 * cos(q0) - vy0 * sin(q0))* Ts;
	double y1 = y0 + (vx0 * sin(q0) + vy0 * cos(q0))* Ts;
	double q1 = q0 + w0 * Ts;
	return {x1, y1, q1};
}

std::vector<double> AcadoMPC::motion_prediction(const std::vector<double> &cur_states,
  const std::vector<std::vector<double>> &prev_u)
{
  std::vector<double> old_vx_cmd = prev_u[0];
  std::vector<double> old_vy_cmd = prev_u[1];
  std::vector<double> old_w_cmd = prev_u[2];

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
    std::vector<double> next_state = update_states(cur_state, old_vx_cmd[i], old_vy_cmd[i], old_w_cmd[i]);
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