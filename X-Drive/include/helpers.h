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

int getSign(double value) {
  if (value == 0) {
    return 1;
  }
  return (fabs(value) / value);
}

class DelayTimer {
  private:
  timer Time;
  double start_time = Time.time();
  double delay_val = 0;
  public:
  void startTimer(double new_delay = 0) {
    if (new_delay != 0)
      delay_val = new_delay;

    start_time = Time.time();
  }
  bool checkTimer() {
    return Time.time() > (start_time + delay_val);
  }
};