#ifndef MATHFUNC_HPP
#define MATHFUNC_HPP 

template <typename T> int sign(T val) { return (T(0) < val) - (val < T(0)); }

double angleWrap(double angle) {
  if (angle >= 18000) {
    angle -= 36000;
  }

  if (angle < -18000) {
    angle += 36000;
  }

  return angle;
}

double angleWrapOneDirection(double angle, int direction, int range=5000, int inversion = 1) {
  if(sign(angle) != direction && range > angle) {
    angle = (angle + 36000*direction) * inversion;
  }
  return angle;
}

#endif