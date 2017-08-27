/*
 * PolyUtil.h
 *
 *  Created on: Aug 22, 2017
 *      Author: deanliu
 */

#ifndef UTILFUNCTIONS_H_
#define UTILFUNCTIONS_H_
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <cppad/cppad.hpp>
//#include <cppad/ipopt/solve.hpp>
#include <cassert>

using CppAD::AD;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// [mph] to [meter per sec] conversion.
// Because I hate having 'magic numbers' spread all over the code base.
// https://en.wikipedia.org/wiki/Magic_number_(programming)#Unnamed_numerical_constants
inline double mph2mps(void) { return 0.44704; }

// Limit angle range within a circle
// Default is (0, 2pi] if upper_limit param not is supplied
// otherwise if specified param = pi, then range becomes (-pi, pi]
inline double wrapAngle(double angle, double upper_limit = 2*M_PI)
{
  double lower_limit = upper_limit - 2*M_PI;
  while (angle > upper_limit)   angle -= 2*M_PI;
  while (angle <=lower_limit)   angle += 2*M_PI;
  return angle;
}

// propagate vehicle state, in MAP COORD SYSTEM
inline std::vector<double> propagate_state_map_coord(std::vector<double> in, double dt)
{
  assert(in.size()==6 && "propagate_state_map_coord() input vector size must equal 6 !!");
  std::vector<double> out;
  double x0     = in[0];
  double y0     = in[1];
  double psi0   = in[2];
  double v0     = in[3];
  double steerLf= in[4];
  double accel  = in[5];
  out.push_back(x0 + v0*cos(psi0)*dt);    // x1
  out.push_back(y0 + v0*sin(psi0)*dt);    // y1
  out.push_back(psi0 + v0*(steerLf)*dt); // psi1
  out.push_back(v0 + accel*dt);           // v1
  return out;
}

// Evaluate a polynomial (overloaded CppAD:: version)
inline AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x)
{
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// Evaluate a polynomial (overloaded std:: version)
inline double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Derivative of a polynomial
inline Eigen::VectorXd polyder(Eigen::VectorXd p)
{
  Eigen::VectorXd k(p.size()-1);
  for (int i = 1; i < p.size(); i++) {
    k[i-1] = (float)i * p[i]; // multiple coeff by its power, and shift 1 down
  }
  return k; // polynomial coefficients
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result; // polynomial coefficients
}


#endif /* UTILFUNCTIONS_H_ */
