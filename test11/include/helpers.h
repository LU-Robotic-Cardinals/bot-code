#include <cmath>
#include <string>
#include <vector>
#include <iostream>

bool closetonum(double value = 0, double target = 0, double error = 0.001) {
    if (fabs(value - target) < fabs(error)) {
        return true;
    }
    return false;
}

double getSign(double value) {
  if (value == 0) {
    return 1;
  }
  return (fabs(value) / value);
}