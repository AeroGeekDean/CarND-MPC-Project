/*
 * Car.h
 *
 *  Created on: Aug 21, 2017
 *      Author: deanliu
 */

#ifndef CAR_H_
#define CAR_H_

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"

class Car {
 public:

  Car();
  virtual ~Car();

  // convert from map to body coordinates
  void map2body(double& x_map, double& y_map,
                double& x_b, double& y_b);

  void calc_nav_errs();

  void update();

  double get_steerCmd() const              { return mySteerCmd; }
  double get_accelCmd() const              { return myAccelCmd; }

  void get_predTraj(std::vector<double>& xs_out, std::vector<double>& ys_out);
  void test_polyder(std::vector<double>& xs_out, std::vector<double>& ys_out);

  MPC& get_mpc() { return myMpc; }

  // Mutator methods
  void set_x(double x_in)                       {this->myX = x_in;}
  void set_y(double y_in)                       {this->myY = y_in;}
  void set_psi(double psi_in)                   {this->myPsi = psi_in;}
  void set_v(double v_in)                       {this->myV = v_in;}
  void setCoeffs(const Eigen::VectorXd& coeffs) {this->coeffs = coeffs;}
  void set_steerFb(double mySteerFb)            { this->mySteerFb = mySteerFb; }
  void set_accelFb(double myAccelFb)            { this->myAccelFb = myAccelFb; }

 private:

  MPC myMpc;  // ModelPredictiveController

  // car's own state (in map coordinate system)
  double myX;             // [m], (+) east
  double myY;             // [m], (+) north
  double myPsi;           // [rad], map axis, (+) CCW from east
  double myV;             // [m/s], (+) forward
  double myCte;           // [m], (+) car is RIGHT of Ref Traj
  double myPsiErr;        // [rad], (+) Ref heading is RIGHT of nose

  double mySteerFb;      // feedback [rad], (+) right turn
  double myAccelFb;      // feedback [ND], (+) fwd, normalized range (+/-1)

  double mySteerCmd;      // [rad], (+) right turn
  double myAccelCmd;      // [ND], (+) fwd, normalized range (+/-1)

  Eigen::VectorXd coeffs; // Ref trajectory polyfitted coeffs, in body axis (x-fwd, y-left)
  Eigen::VectorXd coeffs_der; // coeff of Ref Traj's 1st derivative (ie: slope -> heading)
};

#endif /* CAR_H_ */
