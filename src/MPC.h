#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "FG_eval.h"

//using namespace std; // this is poor software practice!

class MPC {
 public:
  MPC();

  virtual ~MPC();

  void init();
  void set_state(Eigen::VectorXd& state_ic) { this->x0 = state_ic; }
  void set_coeff(Eigen::VectorXd& coeffs) { fg.set_coeff(coeffs); }
  void set_control(Eigen::VectorXd& control_ic) { this->u0 = control_ic; }
  void set_dt(double dt_in) { this->fg.set_dtActual( dt_in ); }

  // Solve the model and return the current actuations (steering and accel)
  // Assumes initial state and polynomial coefficients has already been given.
  void Solve();

  const std::vector<double>& get_output() const { return output; }

  const std::vector<double>& getPredTrajX() const { return pred_traj_x; }
  const std::vector<double>& getPredTrajY() const { return pred_traj_y; }

 private:
/*
  Eigen::VectorXd propagate_model(Eigen::VectorXd& x_in,
                                  Eigen::VectorXd& u_in,
                                  double dt);
*/

  double look_ahead_time;   // time horizon, T [sec]
//  double dt;                // dt, [sec]
//  size_t N;                 // num of frames to time horizon

  FG_eval fg;

  Eigen::VectorXd x0; // inputed state
  Eigen::VectorXd u0; // current control inputs

  std::vector<double> output; // MPC's outputs

  std::vector<double> pred_traj_x;
  std::vector<double> pred_traj_y;

};

#endif /* MPC_H */
