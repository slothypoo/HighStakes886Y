#ifndef MATHFUNC_HPP
#define MATHFUNC_HPP

double angleWrap(double angle) {
  if (angle >= 18000) {
    angle -= 36000;
  }

  if (angle < -18000) {
    angle += 36000;
  }

  return angle;
}

template <typename T> int sign(T val) { return (T(0) < val) - (val < T(0)); }

#endif