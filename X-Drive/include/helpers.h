#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#ifndef HELPERS_H
#define HELPERS_H

int binoCoef(int n, int k) {
      
    // k can not be grater then k so we return 0 here
    if (k > n)
        return 0;
  
      // base condition when k and n are equal or k = 0
    if (k == 0 || k == n)
        return 1;

    // Recurvie add the value 
    return binoCoef(n - 1, k - 1)
           + binoCoef(n - 1, k);
}

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


class ToggleB {
  public:
  // Simple toggle button stuct
  // run setval and update each loop
  bool output = false;
  bool lastUpdatePressed = false;
  
  bool update(bool button) {
    if (button && (!lastUpdatePressed))
      output = !output;
    lastUpdatePressed = button;
    return output;
  }
  bool getValue() {
    return output;
  }
  bool setValue(bool setVal) {
    output = setVal;
    return output;
  }
};


class SingleB {
  public:
  // Simple toggle button stuct
  // run setval and update each loop
  bool output = false;
  bool lastUpdatePressed = false;
  
  bool update(bool button) {
    // static bool lastUpdatePressed = false;
    if (button && (!lastUpdatePressed))
      output = true;
    else
      output = false;
    lastUpdatePressed = button;
    return output;
  }
  bool getValue() {
    return output;
  }
  bool setValue(bool setVal) {
    output = setVal;
    return output;
  }
};


class RollingAverage {
public:
    RollingAverage(int window_size) : window_size_(window_size), sum_(0.0), window_(window_size, 0.0) {}

    void add(double value) {
      sum_ += value - window_[index_];
      window_[index_] = value;
      index_ = (index_ + 1) % window_size_;
    }

    double average() const {
      return sum_ / window_size_;
    }

private:
    int window_size_;
    double sum_;
    std::vector<double> window_;
    int index_ = 0;
};


#endif // HELPERS_H