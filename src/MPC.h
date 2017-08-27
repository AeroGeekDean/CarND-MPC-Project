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

  void set_state(Eigen::VectorXd& state_ic)     { this->x0 = state_ic; }
  void set_coeff(Eigen::VectorXd& coeffs)       { this->fg.set_coeff(coeffs); }
  void set_control(Eigen::VectorXd& control_ic) { this->u0 = control_ic; }
  void set_dt(double dt_in)                     { this->fg.set_dtActual( dt_in ); }

  // Solve the model and calc the current actuations (steering and accel)
  // Assumes initial state and polynomial coefficients has already been given.
  void Solve();

  const std::vector<double>& get_output() const   { return output; }
  const std::vector<double>& get_PredTrajX() const { return pred_traj_x; }
  const std::vector<double>& get_PredTrajY() const { return pred_traj_y; }
  const std::vector<double>& get_psi_vals() const { return pred_psi; }
  const std::vector<double>& get_v_vals() const { return pred_v; }
  const std::vector<double>& get_cte_vals() const { return pred_cte; }
  const std::vector<double>& get_epsi_vals() const { return pred_epsi; }
  const std::vector<double>& get_delta_vals() const { return pred_delta; }
  const std::vector<double>& get_a_vals() const { return pred_a; }


 private:

  double look_ahead_time;   // time horizon, T [sec]

  FG_eval fg; // Class used for the optimizer

  Eigen::VectorXd x0; // current state
  Eigen::VectorXd u0; // current control

  std::vector<double> output; // MPC's outputs

  std::vector<double> pred_traj_x;
  std::vector<double> pred_traj_y;

  std::vector<double> pred_psi;
  std::vector<double> pred_v;
  std::vector<double> pred_cte;
  std::vector<double> pred_epsi;
  std::vector<double> pred_delta;
  std::vector<double> pred_a;
};

#endif /* MPC_H */
