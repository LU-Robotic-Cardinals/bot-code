#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

struct AngledM {
    // Struct variable definitions
    vex::motor M;
    double max_wheel_speed; // Rot speed of wheel at 100% power
    double wheel_radius; // Wheel radius
    double wheel_angle; // Angle from arbitrary "front"
    double dist_from_rot_center; // Radius from center of X to middle of wheel contact patch
    double lin_coef; // Special correcting variable for linear motion
    double rot_coef; // Special correcting variable for rotational motion

    double get_lin_speed_coef(double theta) {
      double val = cos( ((theta - wheel_angle)/180.0) * M_PI) * lin_coef;

      // Perhaps not needed
      // if (fabs(val) < (0.01 * fabs(speed_coef)))
      //   val = 0;

      return val;
    }

    // speed is in inches/min
    // Returns motor speed percentage to achive desired linear speed
    // given steering angle, wheel radius, and max motor rotations/min
    double get_lin_speed(double lin_speed, double theta) {
      // Spin coeficient based on relative angles
      double coef = get_lin_speed_coef(theta);
      // Circumfrence... that's it.
      double wheel_circumference = 2 * M_PI * wheel_radius;

      // Number of revolutions per minute it takes for
      // wheel circumference to travel the dist in one minute
      double revs_per_min = lin_speed / wheel_circumference;

      // This calculates with the following units
      // in/min / in/rot * coef = rot/min of wheel
      // rot/min / rot/min = % motor power
      double speed_percent = (revs_per_min * coef) / max_wheel_speed;
      return speed_percent;
    }

    // This calculates the maximum linear speed that the
    // motor can handle
    double get_max_lin_speed(double theta) {
      // Dummy speed of 60 in/min and use given angle
      // Need to have angle because max speed is infinite
      // at certain angles

      // zero_speed is always zero as far as I can tell
      // it is just here in case that changes in the future
      double zero_speed = get_lin_speed(0, theta);
      double one_speed = get_lin_speed(60, theta);
      // Delta speed % / delta speed
      double slope = (one_speed - zero_speed)/60;

      // Since 100% is the max, find how many "slopes" 
      // it takes to max out the motor
      double result = 1.0 / slope;
      return result;
      
    }

    double get_rot_speed_coef(double rot_speed) {
      if (rot_speed == 0)
        rot_speed = 1;
      return (fabs(rot_speed)/rot_speed) * rot_coef;
    }

    // rot_speed is in deg/sec
    double get_rot_speed(double rot_speed) {

      double coef = get_rot_speed_coef(rot_speed);

      // if (rot_speed == 0)
      //   rot_speed = 1;
      double wheel_circumference = 2 * M_PI * wheel_radius;
      
      // Circumfrence of circle traced out
      // by wheel when the bot spins.
      // double arc_circumference = 2 * M_PI * dist_from_rot_center;

      // Length of arc traced out by wheel
      // in one minute
      // deg/sec * rad/deg * dist/rad = dist/sec
      // dist/sec * 60sec/min * 1 min = dist in one minute

      double arc_length = fabs(rot_speed) * (M_PI / 180.0) * dist_from_rot_center;

      // Revs that the wheel has to perform per minute
      // X 60 for sec to minute conversion
      double revs_per_min = arc_length * 60 / wheel_circumference;

      // rot/min
      double speed_percent = (revs_per_min * coef) / max_wheel_speed;

      return speed_percent;
    }

    // This calculates the maximum angular speed that the
    // motor can handle
    double get_max_rot_speed() {
      // Dummy speed of 1 deg/sec
      // Need to have angle because max speed is infinite
      // at certain angles

      // zero_speed is always zero as far as I can tell
      // it is just here in case that changes in the future
      double zero_speed = get_rot_speed(0);
      double one_speed = get_rot_speed(1);
      // Delta speed % / delta speed
      double slope = (one_speed - zero_speed)/1.0;

      // Since 100% is the max, find how many "slopes" 
      // it takes to max out the motor
      double result = 1.0 / slope;
      return fabs(result);
      
    }

    // This just sets the speed of the motor
    // to a percentage value
    void set_speed(double speed) {
      M.spin(forward,speed*100,percent);
    }
};

class ToggleB {
  public:
  // Simple toggle button stuct
  // run setval and update each loop
  bool output = false;
  bool lastUpdatePressed = false;
  
  bool update(bool button) {
    // static bool lastUpdatePressed = false;
    if (button && (!lastUpdatePressed))
      output = !output;
    lastUpdatePressed = button;
    std::cout << &output << " : output : " << output << "\n";
    std::cout << &lastUpdatePressed << " : last : " << lastUpdatePressed << "\n";
    std::cout << &button << " : button : " << button << "\n";
    std::cout << "Break\n";
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
    static bool lastUpdatePressed = false;
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


class X_Drive {
private:
    std::vector<AngledM> motors;
    double max_motor_speed; // Needs to be set as a percentage 0 <= x <= 1
    double linear_speed; //In inches/min
    double steering_angle; // In degrees
    double rot_speed;

public:
    X_Drive(const std::vector<AngledM>& initialMotors)
        : motors(initialMotors), max_motor_speed(1), linear_speed(0), steering_angle(0), rot_speed(0) {}

    void addMotor(const AngledM& motor) {
        motors.push_back(motor);
    }

    std::vector<AngledM> getMotors() const {
        return motors;
    }

    void removeMotorByIndex(size_t index) {
        if (index < motors.size()) {
            motors.erase(motors.begin() + index);
        }
    }

    void update() {
      // Percent speed for each motor
      std::vector<double> p_speeds(motors.size());

      double max_coef = 0;
      for (size_t i = 0; i < motors.size(); ++i) {
          p_speeds[i]  = motors[i].get_lin_speed(linear_speed,steering_angle);
          p_speeds[i] += motors[i].get_rot_speed(rot_speed);
          if (fabs(p_speeds[i]) > max_coef) {
              max_coef = fabs(p_speeds[i]);
          }
      }

      double overspeed_scalar = 1;              //  \/ \/ \/ \/ \/ This is to make sure the motors don't max out
      if ((fabs(max_coef) > max_motor_speed) || (fabs(max_coef) > 1.0))
          overspeed_scalar = (1.0 / max_coef) * std::min(max_motor_speed, 1.0);

      overspeed_scalar = 1;

      for (size_t i = 0; i < motors.size(); ++i) {
          p_speeds[i] = p_speeds[i] * overspeed_scalar;
          motors[i].set_speed(p_speeds[i]);
          // printf("%7.2f\n",p_speeds[i]);
      }
      // printf("\n");
    }

    double get_max_lin_speed(double angle) {
      // This is the X_Drive class implimentation of the
      // AngledM get_max_lin_speed function for every motor
      // in the group.
      double max_speed = 0;
      for (size_t i = 0; i < motors.size(); ++i) {
        // Find the slowest one since that will imit all the others
        // max_speed == 0  is because the max_speed is initialized to zero
        // and not some massive number. Without it, get_max_lin_speed would
        // not give the right number.
        if (motors[i].get_max_lin_speed(angle) < max_speed || max_speed == 0)
          max_speed = motors[i].get_max_lin_speed(angle);
      }
      return max_speed;
    }

    double get_max_rot_speed() {
      // This is the X_Drive class implimentation of the
      // AngledM get_max_rot_speed function for every motor
      // in the group.
      double max_speed = 0;
      for (size_t i = 0; i < motors.size(); ++i) {
        // Find the slowest one since that will imit all the others
        // (max_speed == 0)  is because the max_speed is initialized to zero
        // and not some massive number. Without it, get_max_rot_speed would
        // not give the right number.
        if (motors[i].get_max_rot_speed() < max_speed || max_speed == 0)
          max_speed = motors[i].get_max_rot_speed();
      }
      return max_speed;
    }

    void set_max_speed(double new_speed) {
        max_motor_speed = new_speed;
    }

    void set_lin_speed(double new_speed) {
        linear_speed = new_speed;
    }

    void set_steeringAngle(double new_angle) {
        steering_angle = new_angle;
    }
    void set_rot_speed(double new_speed) {
        rot_speed = new_speed;
    }
};


class PIDController {
public:
    PIDController(double P, double I, double D) : Kp(P), Ki(I), Kd(D), integral(0.0), previous_error(0.0) {
      last_time = Time.time();
    }

    double calculate(double error) {
        // The divide by 10 is to keep things under controll
        double dt = (Time.time() - last_time)/10; 
        last_time = Time.time();
        double proportional = Kp * error;
        integral += Ki * error * dt;
        double derivative = Kd * (error - previous_error) / dt;
        previous_error = error;

        return proportional + integral + derivative;
    }
    
    std::vector<double> values(){
      double dt = (Time.time() - last_time)/10;
      double prop = Kp * previous_error;
      double inte = Ki * previous_error * dt;
      double deri = Kd * (previous_error - previous_error) / dt;
      std::vector<double> array = {prop, inte, deri, previous_error};
      return array;
    }

private:
    double Kp, Ki, Kd;
    double integral, previous_error;
    double last_time;
    timer Time;
};
