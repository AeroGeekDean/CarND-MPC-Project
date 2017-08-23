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

  calc_nav_errs();

  // update MPC with current vehicle states
  Eigen::VectorXd state(6);
  state << myPos.x, myPos.y, myPsi, myV, myCte, myPsiErr;
  myMpc.updata_state(state);

  myMpc.update_coeff(coeffs);

  auto outputs = myMpc.Solve();

  mySteerCmd = outputs[0];
  myAccelCmd = outputs[1];

}

void Car::calc_nav_errs() {
  // Note- vehicle body coordinate system has origin at the vehicle
  // Thus car's (x, y) = (0, 0)

  // calculate the cross track error

  myCte = polyeval(coeffs, 0.0); // Note: polynomial is in body coord already

  // calculate the orientation error
  coeffs_der = polyder(coeffs); // 1st derivative coeff
  double refPsi_body = atan(polyeval(coeffs_der, 0.0)); // tangent angle of slope
  refPsi = wrapAngle(myPsi - refPsi_body);  // convert from body to map axis
  myPsiErr = wrapAngle((refPsi - myPsi), M_PI); // for control system, use +/-pi
}

void Car::predTraj(std::vector<double>& xs_out, std::vector<double>& ys_out) {
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
  xs_out.push_back(ahead_x-seg_len);
  ys_out.push_back( ahead_y - ahead_slope * (ahead_x-seg_len) );

  xs_out.push_back(ahead_x);
  ys_out.push_back(ahead_y);

  xs_out.push_back(ahead_x+seg_len);
  ys_out.push_back( ahead_y + ahead_slope * (ahead_x+seg_len) );
}
