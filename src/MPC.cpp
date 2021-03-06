#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//
// MPC class definition implementation.
//
MPC::MPC() {
  // set some default value, to safe guard against forgetting to call init()...
  look_ahead_time = 5.0; // [sec]
  fg.dt_model = 0.1; // 10 Hz [sec]
  fg.N = (size_t) (look_ahead_time/fg.dt_model);
}

MPC::~MPC() {}

void MPC::init() {

  look_ahead_time = 1.5; // [sec]
  fg.dt_model = 0.1; // 10 Hz [sec]
  fg.N = (size_t) (look_ahead_time/fg.dt_model);

  fg.vref = 55*mph2mps(); //                <----   set reference speed, [mph]

  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we need to establish
  // when one variable starts and another ends to make our life easier.
  fg.x_start     = 0;
  fg.y_start     = fg.x_start + fg.N;
  fg.psi_start   = fg.y_start + fg.N;
  fg.v_start     = fg.psi_start + fg.N;
  fg.cte_start   = fg.v_start + fg.N;
  fg.epsi_start  = fg.cte_start + fg.N;

  fg.delta_start = fg.epsi_start + fg.N;
  fg.a_start     = fg.delta_start + (fg.N - 1); // controls start 1 frame after init states
}


void MPC::Solve() {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // setup parameters
  size_t num_states = 6;
  size_t num_controls = 2;

  // assign x0 & u0 to local for code clarity
  double x     = x0[0];
  double y     = x0[1];
  double psi   = x0[2];
  double v     = x0[3];
  double cte   = x0[4];
  double epsi  = x0[5];

  double delta = u0[0];
  double accel = u0[1];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = num_states * fg.N + num_controls * (fg.N - 1);

  // Set the number of constraints
  size_t n_constraints = num_states * fg.N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
    vars[i] = 0;

  // Set the init condition variable values
  vars[fg.x_start]    = x;
  vars[fg.y_start]    = y;
  vars[fg.psi_start]  = psi;
  vars[fg.v_start]    = v;
  vars[fg.cte_start]  = cte;
  vars[fg.epsi_start] = epsi;

  fg.delta0           = delta;  // <---- Added previous frame's controls as well. So able to penalize for large deviation.
  fg.accel0           = accel;  //       These were missing in classroom!!!

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (int i = 0; i < fg.delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // The upper and lower limits of remaining delta (steering) are
  // (-25, 25) [deg] (apply values in [rad]).
  for (int i = fg.delta_start; i < fg.a_start; i++) {
    vars_lowerbound[i] = deg2rad(-25)*fg.Lf;
    vars_upperbound[i] = deg2rad(25)*fg.Lf;
  }

  // Upper and lower limits of remaining acceleration
  // [-1,1] normalized, [m/s^2]
  for (int i = fg.a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[fg.x_start]     = x;
  constraints_lowerbound[fg.y_start]     = y;
  constraints_lowerbound[fg.psi_start]   = psi;
  constraints_lowerbound[fg.v_start]     = v;
  constraints_lowerbound[fg.cte_start]   = cte;
  constraints_lowerbound[fg.epsi_start]  = epsi;

  constraints_upperbound[fg.x_start]     = x;
  constraints_upperbound[fg.y_start]     = y;
  constraints_upperbound[fg.psi_start]   = psi;
  constraints_upperbound[fg.v_start]     = v;
  constraints_upperbound[fg.cte_start]   = cte;
  constraints_upperbound[fg.epsi_start]  = epsi;

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;

  // <--- would be nice if status of MPC could be shown on simulator dash board... --->

  pred_traj_x.clear();
  pred_traj_y.clear();
//  pred_psi.clear(); // debug/testing use
//  pred_v.clear();
//  pred_cte.clear();
//  pred_epsi.clear();
//  pred_delta.clear();
//  pred_a.clear();

  // Save off predicted trajectory for MPC plotting
  for (int i=0; i<fg.N; i++) {
    pred_traj_x.push_back(solution.x[fg.x_start + i]);
    pred_traj_y.push_back(solution.x[fg.y_start + i]);

    // Save MPC's predicted states, for debug purpose
//    pred_psi.push_back(solution.x[fg.psi_start + i]);
//    pred_v.push_back(solution.x[fg.v_start + i]);
//    pred_cte.push_back(solution.x[fg.cte_start + i]);
//    pred_epsi.push_back(solution.x[fg.epsi_start + i]);
//
//    // Controls have 1 less element. Thus repeat fill last element
//    if (i==(fg.N-1)) {
//      pred_delta.push_back(solution.x[fg.delta_start + i -1]);
//      pred_a.push_back(solution.x[fg.a_start + i -1]);
//    }
//    else {
//      pred_delta.push_back(solution.x[fg.delta_start + i]);
//      pred_a.push_back(solution.x[fg.a_start + i]);
//    }
  }

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  output.clear();
  output.push_back(solution.x[fg.delta_start]);
  output.push_back(solution.x[fg.a_start]);

//  std::cout << (ok ? "OK " : "FAIL ")
//            << ", Cost " << cost
//            << ", Steer " << output[0]
//            << ", Accel " << output[1]
//            << std::endl;
  return;
}
