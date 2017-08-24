#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
//const double Lf = 2.67;

//
// MPC class definition implementation.
//
MPC::MPC() {
  look_ahead_time = 5.0; // [sec]
  dt = 0.1; // 10 Hz [sec]
  N = (int) look_ahead_time/dt;
}

MPC::~MPC() {}

void MPC::init() {
  look_ahead_time = 3.0; // [sec]
  dt = 0.1; // 10 Hz [sec]
  N = (int) look_ahead_time/dt;

  /* Not the cleanest OOP design to make these FG_eval members public
   * but they need to be used by both MPC and FG_eval classes.
   * Perhaps making them friend classes might be an option.
   * But I haven't practiced using C++ 'friendship' feature yet...
   */
  fg.dt = dt;
  fg.N = N;

  fg.vref = 15; // set reference speed, [mph]

  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we need to establish
  // when one variable starts and another ends to make our life easier.
  fg.x_start     = 0;
  fg.y_start     = fg.x_start + N;
  fg.psi_start   = fg.y_start + N;
  fg.v_start     = fg.psi_start + N;
  fg.cte_start   = fg.v_start + N;
  fg.epsi_start  = fg.cte_start + N;

  fg.delta_start = fg.epsi_start + N;
  fg.a_start     = fg.delta_start + (N-1); // controls start 1 frame after init states

}

void MPC::Solve(double& steer_cmd_out, double& accel_cmd_out) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // setup parameters
  size_t num_states = 6;
  size_t num_controls = 2;

  // assign x0 to local for code clarity
  double x    = x0[0];
  double y    = x0[1];
  double psi  = x0[2];
  double v    = x0[3];
  double cte  = x0[4];
  double epsi = x0[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4*10 + 2*(10-1)
  size_t n_vars = num_states * N + num_controls * (N-1);

  // Set the number of constraints
  size_t n_constraints = num_states * N;

//  std::cout << "n_vars = " << n_vars << " n_constraints = " << n_constraints << std::endl;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) { vars[i] = 0; }
  // Set the init condition variable values
  vars[fg.x_start]     = x;
  vars[fg.y_start]     = y;
  vars[fg.psi_start]   = psi;
  vars[fg.v_start]     = v;
  vars[fg.cte_start]   = cte;
  vars[fg.epsi_start]  = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (int i = 0; i < fg.delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }
  // The upper and lower limits of delta (steering) are
  // (-25, 25) [deg] (apply values in [rad]).
  // NOTE: Feel free to change this to something else.
  for (int i = fg.delta_start; i < fg.a_start; i++) {
    vars_lowerbound[i] = -0.436332; // -25*pi/180
    vars_upperbound[i] =  0.436332; //  25*pi/180
  }
  // Acceleration upper and lower limits.
  // [mph/sec]
  // NOTE: Feel free to change this to something else.
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

  //  // object that computes objective and constraints
//    FG_eval fg_eval(coeffs); // it's now a private member of MPC class

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

  // Save off predicted trajectory for MPC plotting
  pred_traj_x.clear();
  pred_traj_y.clear();

  for (int i=1; i < fg.N; i++) {
    pred_traj_x.push_back(solution.x[fg.x_start+i]);
    pred_traj_y.push_back(solution.x[fg.y_start+i]);
  }

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  steer_cmd_out = solution.x[fg.delta_start];
  accel_cmd_out = solution.x[fg.a_start];

  std::cout << (ok ? "OK " : "FAIL ")
            << ", Cost " << cost
            << ", Steer " << steer_cmd_out
            << ", Accel " << accel_cmd_out
            << ", cte " << cte
            << ", epsi " << rad2deg(epsi)
            << std::endl;

  return;
}
