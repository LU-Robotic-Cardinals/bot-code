#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include "helpers.h"

using namespace vex;

#ifndef DRIVE_H
#define DRIVE_H

struct AngledM {
  private:
    // Struct variable definitions
    vex::motor Motor;
    // In rot/min
    double max_motor_speed; // Rot speed of motor at 100% power
    double max_wheel_speed; // Rot speed of wheel at 100% power
    // Inches
    double wheel_radius; // Wheel radius
    double wheel_angle; // Angle from arbitrary "front"
    double dist_rot_center; // Radius from center of X to middle of wheel contact patch
    double lin_coef; // Special correcting variable for linear motion
    double rot_coef; // Special correcting variable for rotational motion

    // For PID stuff
    // PIDController pid_obj;
    timer Time; // Time object
    double current_speed = 0; // Current speed, updated on set_speed()
    double current_time = Time.time(); // Global deffinition of current time
    double last_time = current_time; // Time on last update
    double last_rotation = 0; // Cummulative position for motor to achive


  public:


    // This just sets the speed of the motor
    // to a percentage value
    void set_speed(double new_speed) {
      // std::cout << new_speed << "\n";
      // new_speed = 1;
      // current_speed = 0;
      // motor.spin(forward,new_speed*100,percent);
      // Delta time
      double dt = (Time.time() - last_time)/1000.0;

      // delta speed
      // change in speed per time
      long double dv = (new_speed - current_speed) / dt;

      // Maximum acceleration
      // per second
      double max_accell = 1/10;
      // If acceleration is larger than limit
      // and limiting is enabled, then apply
      // the limit.
      if (fabs(dv) > fabs(max_accell * dt) && max_accell != 0) {
        dv = fabs(max_accell * dt) * getSign(dv);
      }

      if (getSign(new_speed - (current_speed + dv)) == -getSign(new_speed - current_speed))
        current_speed = new_speed;
      else {
      // Calculate speed
        current_speed += dv;
      }

      // if (fabs(current_speed) > 1)
      //   current_speed = fabs(current_speed) / current_speed;
      // current_speed = new_speed;

      Motor.spin(forward,current_speed*100,percent);

      last_time = Time.time();
    }




    AngledM(vex::motor Motor_Obj, double maximum_motor_speed = 200,
    double maximum_wheel_speed = 200, double wheel_radius_size = 1, 
    double wheel_center_angle = 0, double wheel_bot_radius = 1, 
    double linear_coefficient = 1, double rotation_coefficient = 1) :

    Motor(Motor_Obj), max_motor_speed(maximum_motor_speed),
    max_wheel_speed(maximum_wheel_speed), wheel_radius(wheel_radius_size),
    wheel_angle(wheel_center_angle), dist_rot_center(wheel_bot_radius), 
    lin_coef(linear_coefficient), rot_coef(rotation_coefficient) {}

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
      return fabs(result);
      
    }

    double get_rot_speed_coef(double rot_speed) {
      // You can't divide by zero dummy
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

      double arc_length = fabs(rot_speed) * (M_PI / 180.0) * dist_rot_center;

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
    // std::cout << max_coef << "\n";

    double overspeed_scalar = 1;              //  \/ \/ \/ \/ \/ This is to make sure the motors don't max out
    if ((fabs(max_coef) > max_motor_speed) || (fabs(max_coef) > 1.0))
      overspeed_scalar = (1.0 / max_coef) * std::min(max_motor_speed, 1.0);

    // overspeed_scalar = 1;

    for (size_t i = 0; i < motors.size(); ++i) {
      p_speeds[i] = p_speeds[i] * overspeed_scalar;
      motors[i].set_speed(p_speeds[i]);
      // std::cout << p_speeds[i] << "\n\n\n";
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


double wheel_rad_size = 1.625; // In inches
double bot_radius = 10;

double motor_reference = 0; // Degrees from brain
double motor_speed = 200;
double wheel_speed = 200 * 5 / 3;
std::vector<AngledM> x_motor_constructor(std::vector<vex::motor> motors, int motors_per_axle, 
  double wheel_rad_size, double bot_radius, double motor_speed, double angle_reference = 0, double gear_ratio = 1,
  double global_lin_coef = 1, double global_rot_coef = 1) {

  std::vector<AngledM> initialMotors;
  int axle = 0;
  for (int i = 0; i < motors.size(); i++) {
    double angle = ( 90 * (axle) - 45);
    if (angle > 180)
     angle -= 360;
    int lin_coef = pow(-1,i % motors_per_axle + axle + 1);
    int rot_coef = pow(-1,i % motors_per_axle);

    initialMotors.push_back(
      AngledM(motors[i], motor_speed, motor_speed * gear_ratio, wheel_rad_size, angle-angle_reference,
       bot_radius, lin_coef * global_lin_coef, rot_coef * global_rot_coef)
    );
    if (motors_per_axle == 1)
      axle +=1;
    axle += i % motors_per_axle; // Increment the axel val after using it


    // {motor(PORT19, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius, -1,  1}, // NW Top
    // {motor(PORT20, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius,  1, -1}, // NW Dow

    // {motor(PORT2,  ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius,  1,  1}, // NE Top
    // {motor(PORT1,  ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius, -1, -1}, // NE Dow

    // {motor(PORT11, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius, -1,  1}, //SE Top
    // {motor(PORT12, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius,  1, -1}  //SE Dow

    // {motor(PORT10, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius,  1,  1}, // SW Top
    // {motor(PORT9,  ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius, -1, -1}, // SW Dow
  }
  return initialMotors;
}

#endif // DRIVE_H