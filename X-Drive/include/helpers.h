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


class ExclusiveB {
  public:
  // Set of exclusive toggle buttons
  // only one can be active, toggling one decativates all others
  // toggling an active deactivates all others too
  // run setval and update on each loop
  std::vector<ToggleB> button_list;

  ExclusiveB(int num_buttons){
    for (int i = 0; i < num_buttons; i++){
      ToggleB new_button;
      button_list.push_back(new_button);
    }
  }
  
  bool update(int button_index, bool button_val) {
    bool old_output = button_list[button_index].getValue();
    bool output;
    output = button_list[button_index].update(button_val);
    // Itterate through all the buttons
    if (old_output != output)
    for (int i = 0; i < button_list.size(); i++){
      // If the button is not the one we just changed
      // then set it to false as per the rules above.
      if (i != button_index)
        button_list[i].setValue(false);
    }
    return output;
  }

  bool getValue(int button_index) {
    return button_list[button_index].getValue();
  }

  // Fix later? Do I want setting a button to false to set
  // all to false or not affect any other ones?
  bool setValue(int button_index, bool setVal) {
    bool output;
    output = button_list[button_index].setValue(setVal);
    // Itterate through all the buttons
    for (int i = 0; i < button_list.size(); i++){
      // If the button is not the one we just changed
      // then set it to false as per the rules above.
      if (i != button_index)
        button_list[i].setValue(false);
    }
    return output;
  }
};


class PulseB {
  public:
  // Simple pulse button stuct
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


int getRotationDirection(double targetAngle, double gyroAngle) {
  // Calculate the angle difference, ensuring it's within -180 to 180 degrees
  float angleDiff = targetAngle - gyroAngle;
  while (angleDiff > 180.0) {
    angleDiff -= 360.0;
  }
  while (angleDiff <= -180.0) {
    angleDiff += 360.0;
  }

  // Determine the rotation direction
  if (angleDiff > 0.0) {
    return -1; // Clockwise rotation
  } else if (angleDiff < 0.0) {
    return 1; // Counter-Clockwise rotation
  } else {
    return 0; // No rotation needed
  }
}


#endif // HELPERS_H