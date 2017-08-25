/*
 * FGeval.h
 *
 *  Created on: Aug 23, 2017
 *      Author: deanliu
 */

#ifndef FG_EVAL_H_
#define FG_EVAL_H_

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "UtilFunctions.h"

using CppAD::AD;

class FG_eval {
 public:

  FG_eval() {};
  virtual ~FG_eval() {};

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  size_t N; // num of frames
  double dt; // time step, [sec]
  double vref; // reference speed, [m/s]
  // This is the length from front to CoG that has a similar radius.
  const double Lf = 2.67;

  Eigen::VectorXd coeffs; // Fitted polynomial coefficients

  // state array indices
  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;

  // control array indices
  size_t delta_start;
  size_t a_start;

  void set_coeff(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2)*10;
      fg[0] += CppAD::pow(vars[v_start + t] - vref, 2);
    }

    // Minimize the use of actuators.
//    for (int t = 0; t < N - 1; t++) {
//      fg[0] += CppAD::pow(vars[delta_start + t], 2)*0.01;
//      fg[0] += CppAD::pow(vars[a_start + t], 2); // don't penalize accel/throttle amplitude
//    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2) *10;
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start]     = vars[x_start];
    fg[1 + y_start]     = vars[y_start];
    fg[1 + psi_start]   = vars[psi_start];
    fg[1 + v_start]     = vars[v_start];
    fg[1 + cte_start]   = vars[cte_start];
    fg[1 + epsi_start]  = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x0     = vars[x_start + t - 1];
      AD<double> y0     = vars[y_start + t - 1];
      AD<double> psi0   = vars[psi_start + t - 1];
      AD<double> v0     = vars[v_start + t - 1];
//      AD<double> cte0   = vars[cte_start + t - 1]; // don't care about these. use direct method below
//      AD<double> epsi0  = vars[epsi_start + t -1]; // don't care about these. use direct method below

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0     = vars[a_start + t - 1];

      AD<double> x1     = vars[x_start + t];
      AD<double> y1     = vars[y_start + t];
      AD<double> psi1   = vars[psi_start + t];
      AD<double> v1     = vars[v_start + t];
      AD<double> cte1   = vars[cte_start + t];
      AD<double> epsi1  = vars[epsi_start + t];

      AD<double> y1_desired = polyeval(coeffs, x1);
      AD<double> psi_desired = -CppAD::atan(polyeval(polyder(coeffs), x1)); // note: (-1) sign. +slope = -psi !!

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is so that CppAD can compute derivatives and pass
      // these to the solver.

      // TODO: Setup the rest of the model constraints
      // NOTE - *** This is in vehicle body axis!!! ***
      // (+) x      = fwd
      // (+) y      = left
      // (+) psi    = right of nose
      // (+) v      = fwd
      // (+) delta  = right turn
      // (+) cte    = car is right of Ref Trajectory
      // (+) epsi   = nose needs to turn right
      fg[1 + x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]     = y1 - (y0 - v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t]   = psi1 - (psi0 + v0*(delta0/Lf)*dt);
      fg[1 + v_start + t]     = v1 - (v0 + a0*dt);

      // Udacity solution. propagating errors from prev timestep. WHY?!?
//      fg[1 + cte_start + t]   = cte1 - ( (y_desired - y0) + v0 * CppAD::sin(psi0) * dt);
//      fg[1 + epsi_start + t]  = epsi1 - ( (psi_desired - epsi0) + v0/Lf*delta0*dt);

      // My "direct method" working solution
      fg[1 + cte_start + t]   = cte1 - (y1_desired - y1);
      fg[1 + epsi_start + t]  = epsi1 - (psi_desired - psi1);
    } // for loop

  } // Operator()

}; // Class FG_eval


#endif /* FG_EVAL_H_ */
