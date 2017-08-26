/*
 * Car.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: deanliu
 */

#include <math.h>
#include "Car.h"

#include "UtilFunctions.h"

Car::Car() {
  myMpc.init();
}

Car::~Car() {
}

Car::Coord Car::map2body(Car::Coord& in_map) {
  double dxm = in_map.x - myPos.x;
  double dym = in_map.y - myPos.y;
  Coord out_body;
  out_body.x = dxm*cos(myPsi) + dym*sin(myPsi);
  out_body.y =-dxm*sin(myPsi) + dym*cos(myPsi);
  return out_body;
}

void Car::update() {

  // update MPC with current vehicle states
  Eigen::VectorXd state(6);

  // Note - since we're doing everything in vehicle axis
  // x, y, and psi are ALL 0.0 since the car is at the origin
  state << 0.0,     // x pos, [m]
           0.0,     // y pos, [m]
           0.0,     // psi, [rad]
           myV,     // speed, [m/s]
           myCte,   // [m], + to left of RefTraj
           myPsiErr;// psi_err, [rad] + to left of ref Psi
  myMpc.set_state(state);

  myMpc.set_coeff(coeffs); // Ref Trajectory

  Eigen::VectorXd prev_control(2);
  prev_control << mySteerFb,
                  myAccelFb;
  myMpc.set_control(prev_control);

  myMpc.Solve(); // the parameters are the OUTPUTS! (passed by ref)

  std::vector<double> out = myMpc.get_output();
  mySteerCmd = out[0];
  myAccelCmd = out[1];
}

void Car::calc_nav_errs() {
  // Note- vehicle body coordinate system has origin at the vehicle
  // Thus car's (x, y) = (0, 0)

  // calculate the cross track error

  myCte = polyeval(coeffs, 0.0); // Note: polynomial is in body coord already

  // calculate the orientation error
  /* Below is the full equation. but since we're evaluating @ x=0, it simplifies to above */
//  coeffs_der = polyder(coeffs); // 1st derivative coeff
//  myPsiErr = -atan(polyeval(coeffs_der, 0.0)); // tangent angle of slope
  double psi_ref_body = -atan(coeffs[1]); // tangent angle of slope
  myPsiErr = psi_ref_body - 0; // psi of ownship in body axis == 0

  refPsi_map = wrapAngle(myPsi - myPsiErr);  // convert from body to map axis

//  double refPsi_body = -atan(polyeval(coeffs_der, 0.0)); // tangent angle of slope
//  refPsi = wrapAngle(myPsi - refPsi_body);  // convert from body to map axis
//  myPsiErr = wrapAngle((refPsi - myPsi), M_PI); // for control system, use +/-pi
}

void Car::get_predTraj(std::vector<double>& xs_out, std::vector<double>& ys_out) {
  xs_out = myMpc.getPredTrajX();
  ys_out = myMpc.getPredTrajY();
}

void Car::test_polyder(std::vector<double>& xs_out, std::vector<double>& ys_out) {
  /*
   * This method draws 3 dots that are tangent to the Ref Traj at distance `ahead_x`
   * Supply the method with the x,y vectors for the MPC trajectory to visualize.
   * For testing the functions in PolyUtil.h are working properly.
   */
  double seg_len = 10.0;
  double ahead_x = 20.0; // how far ahead of car to calc tangent
  double ahead_y = polyeval(coeffs, ahead_x);
  double ahead_slope = polyeval(coeffs_der, ahead_x);

  // draw 3 dots (tangent to Ref Traj at ahead_x distance)
  xs_out.push_back(ahead_x - seg_len);
  ys_out.push_back(ahead_y - ahead_slope*seg_len);

  xs_out.push_back(ahead_x);
  ys_out.push_back(ahead_y);

  xs_out.push_back(ahead_x + seg_len);
  ys_out.push_back(ahead_y + ahead_slope*seg_len);
}
