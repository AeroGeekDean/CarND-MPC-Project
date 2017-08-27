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
  myMpc.init(); // don't forget this again!
}

Car::~Car() {
}

void Car::map2body(double& x_map, double& y_map,
                   double& x_b, double& y_b) {
  double dxm = x_map - myX;
  double dym = y_map - myY;
  x_b = dxm*cos(myPsi) + dym*sin(myPsi);
  y_b =-dxm*sin(myPsi) + dym*cos(myPsi);
}

void Car::update() {
  // Note - since we're doing everything in vehicle axis
  // x, y, and psi are ALL 0.0 since the car is at the origin
  Eigen::VectorXd state(6);
  state << 0.0,     // x pos, [m]
           0.0,     // y pos, [m]
           0.0,     // psi, [rad]
           myV,     // speed, [m/s]
           myCte,   // [m], + to right of RefTraj
           myPsiErr;// psi_err, [rad] + to left of ref Psi
  myMpc.set_state(state);

  myMpc.set_coeff(coeffs); // Ref Trajectory

  Eigen::VectorXd prev_control(2);
  prev_control << mySteerFb,
                  myAccelFb;
  myMpc.set_control(prev_control);

  myMpc.Solve();

  std::vector<double> out = myMpc.get_output();
  mySteerCmd = out[0];
  myAccelCmd = out[1];
}

void Car::calc_nav_errs() {
  // Note- vehicle body coordinate system has origin at the vehicle
  // Thus car's (x, y) = (0, 0)

  // calculate the cross track error
//  myCte = polyeval(coeffs, 0.0);
  myCte = coeffs[0];

  // calculate the orientation error
  //coeffs_der = polyder(coeffs); // 1st derivative coeff
  //myPsiErr = -atan(polyeval(coeffs_der, 0.0)); // tangent angle of slope
/* Above is the full equation. but since we're evaluating @ x=0, it simplifies to below*/
  myPsiErr = -atan(coeffs[1]); // psi of ownship in body axis == 0
}

void Car::get_predTraj(std::vector<double>& xs_out, std::vector<double>& ys_out) {
  xs_out = myMpc.get_PredTrajX();
  ys_out = myMpc.get_PredTrajY();
}


/*
 * This method draws 3 dots that are tangent to the Ref Traj at distance `ahead_x`
 * Supply the method with the x,y vectors for the MPC trajectory to visualize.
 * For testing the functions in PolyUtil.h are working properly.
 */
void Car::test_polyder(std::vector<double>& xs_out, std::vector<double>& ys_out) {
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
