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
  void set_state(Eigen::VectorXd& ic_state) { this->x0 = ic_state; }
  void set_coeff(Eigen::VectorXd& coeffs) { fg.set_coeff(coeffs); }

  // Solve the model and return the current actuations (steering and accel)
  // Assumes initial state and polynomial coefficients has already been given.
  void Solve(double& steer_cmd_out, double& accel_cmd_out);

  const std::vector<double>& getPredTrajX() const { return pred_traj_x; }
  const std::vector<double>& getPredTrajY() const { return pred_traj_y; }

 private:

  double look_ahead_time;   // time horizon, T [sec]
  double dt;                // dt, [sec]
  size_t N;                 // num of frames to time horizon

  FG_eval fg;

  Eigen::VectorXd x0;

  std::vector<double> pred_traj_x;
  std::vector<double> pred_traj_y;

};

#endif /* MPC_H */
