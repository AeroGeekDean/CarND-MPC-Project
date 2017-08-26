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

  struct Coord {
    double x;
    double y;
  };

  Car();
  virtual ~Car();

  void calc_nav_errs();

  void update();

  double get_steerCmd() const              { return mySteerCmd; }
  double get_accelCmd() const              { return myAccelCmd; }

  void get_predTraj(std::vector<double>& xs_out, std::vector<double>& ys_out);
  void test_polyder(std::vector<double>& xs_out, std::vector<double>& ys_out);

  // convert from map to body coordinates
  Coord map2body(Car::Coord& in_map);

  // Accessor methods
  const Car::Coord& get_location() const    {return myPos;}
  const double get_psi() const              {return myPsi;}
  const double get_v() const                {return myV;}
  const Eigen::VectorXd& getCoeffs() const  {return coeffs;}

  // Mutator methods
  void set_location(Car::Coord& loc_in)         {this->myPos = loc_in;}
  void set_psi(double psi_in)                   {this->myPsi = psi_in;}
  void set_v(double v_in)                       {this->myV = v_in;}
  void setCoeffs(const Eigen::VectorXd& coeffs) {this->coeffs = coeffs;}
  void set_steerFb(double mySteerFb) { this->mySteerFb = mySteerFb; }
  void set_accelFb(double myAccelFb) { this->myAccelFb = myAccelFb; }
  void set_dt(double dt_in) { this->myMpc.set_dt( dt_in ); }


 private:

  MPC myMpc;  // ModelPredictiveController

  // car's own state (in map coordinate system)
  Coord myPos;            // [m], (+) east/north
  double myPsi;           // [rad], map axis, (+) CCW from east
  double myV;             // [m/s], (+) forward
  double myCte;           // [m], (+) car is LEFT of Ref Traj
  double myPsiErr;        // [rad], (+) car pointing LEFT of Ref heading

  double mySteerFb;      // feedback [rad], (+) right turn
  double myAccelFb;      // feedback [ND], (+) fwd, normalized range (+/-1)

  double mySteerCmd;      // [rad], (+) right turn
  double myAccelCmd;      // [ND], (+) fwd, normalized range (+/-1)

  Eigen::VectorXd coeffs; // Ref trajectory polyfitted coeffs, in body axis
  Eigen::VectorXd coeffs_der; // coeff of Ref Traj's 1st derivative (ie: slope)
  double refPsi_map;      // Ref heading in map axis
};

#endif /* CAR_H_ */
