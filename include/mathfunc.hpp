#ifndef MATHFUNC_HPP
#define MATHFUNC_HPP
#include <iostream>

template <typename T> int sign(T val) { return (T(0) < val) - (val < T(0)); }

double angleWrap(double target, double angle) {
  double error = target - angle;
  if (error >= 18000) {
    error -= 36000;
  }
  if (error < -18000) {
    error += 36000;
  }

  if (angle < 10000 && angle > 0) {
    error = error -36000; std::cout << "pls work" << std::endl;
  }
  return error;
}

double angleWrapOneDirection(double target, double angle, int direction,
                             int range = 5000, int inversion = 1) {
  double error = target - angle;
  if (sign(error) != direction && range > angle) {
    error = (error + 36000 * direction) * inversion;
  }
  return error;
}

#endif