/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  Clawbot Template (Drivetrain, No Gyro)                    */
/*                                                                            */
/*    Name:                                                                   */
/*    Date:                                                                   */
/*    Class:                                                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 10           
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

timer Timer;


using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

struct AngledM {
    // Struct variable definitions
    vex::motor M;
    double max_wheel_speed; // Rot speed of wheel at 100% power
    double wheel_radius; // Wheel radius
    double wheel_angle; // Angle from arbitrary "front"
    double dist_from_rot_center; // Radius from center of X to center of wheel
    double lin_coef; // Special correcting variable for linear motion
    // double rot_coef; // Special correcting variable for rotational motion

    double get_speed_coef(double theta) {
      double val = cos( ((theta - wheel_angle)/180) * M_PI) * lin_coef;

      // Perhaps not needed
      // if (fabs(val) < (0.01 * fabs(speed_coef)))
      //   val = 0;

      return val;
    }

    // speed is in inches/min
    // Returns motor speed percentage to achive desired linear speed
    // given steering angle, wheel radius, and max motor rotations/sec
    double get_lin_speed(double speed, double theta) {
      // Spin coeficient based on relative angles
      double coef = get_speed_coef(theta);
      // Circumfrence... that's it.
      double circumference = 2 * M_PI * wheel_radius;

      // This calculates with the following units
      // in/min / in/rot * coef = rot/min of wheel
      // rot/min / rot/min = % motor power
      double speed_percent = ((speed/circumference) * coef) / max_wheel_speed;
      return speed_percent;
    }

    // This calculates the maximum speed that the
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

    // double get_rot_coef(double rot_speed) {
    //   if (rot_speed == 0)
    //     rot_speed = 1;
    //   return (fabs(rot_speed)/rot_speed) * rot_coef;
    // }

    void set_speed(double speed) {
      M.spin(forward,speed,percent);
    }
};

struct ToggleB {
  bool output = false;
  bool lastUpdatePressed = false;
  
  bool update(bool button) {
    static bool lastUpdatePressed = false;
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


class X_Drive {
private:
    std::vector<AngledM> motors;
    double max_motor_speed; // Needs to be set as a percentage 0 <= x <= 1
    double angle_adjust;
    double linear_speed; //In inches/min
    double steering_angle; // In degrees

public:
    X_Drive(const std::vector<AngledM>& initialMotors)
        : motors(initialMotors), max_motor_speed(1), angle_adjust(0), linear_speed(0), steering_angle(0) {}

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
          p_speeds[i] = motors[i].get_lin_speed(linear_speed,steering_angle);
          if (fabs(p_speeds[i]) > max_coef) {
              max_coef = fabs(p_speeds[i]);
          }
      }

      double overspeed_scalar = 1;              //  \/ \/ \/ \/ \/ This is to make sure the motors don't max out
      if ((fabs(max_coef) > max_motor_speed) || (fabs(max_coef) > 1.0))
          overspeed_scalar = (1.0 / max_coef) * std::min(max_motor_speed, 1.0);

      for (size_t i = 0; i < motors.size(); ++i) {
          p_speeds[i] = p_speeds[i] * overspeed_scalar;
          motors[i].set_speed(p_speeds[i]);
      }
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
};

// AngledM NW = {motor(PORT17, ratio18_1, false),  -45,   1, -1};
// AngledM NE = {motor(PORT18, ratio18_1, false),   45,  -1, -1};
// AngledM SW = {motor(PORT20, ratio18_1, false), -135,  -1, -1};
// AngledM SE = {motor(PORT19, ratio18_1, false),  135,   1, -1};

// Wheel radius
double wheel_rad_size = 4.173 / 2; // In inches


// For motor speed, it is written for direct-drive of wheels
// i.e. 100 for red, 200 for green, and 600 for blue
// and can be adjusted to account for gear ratios that are 
// external to the motor
AngledM NW = {motor(PORT17, ratio18_1, false), wheel_rad_size, 200,  -45, 4,  1};
AngledM NE = {motor(PORT18, ratio18_1, false), wheel_rad_size, 200,   45, 4, -1};
AngledM SW = {motor(PORT20, ratio18_1, false), wheel_rad_size, 200, -135, 4, -1};
AngledM SE = {motor(PORT19, ratio18_1, false), wheel_rad_size, 200,  135, 4,  1};

// Array of motors to keep track of them
// Not really a need to define the intermediary
// variables (NW, NE, etc.) but doing so for now
// to help with debugging.
std::vector<AngledM> initialMotors = {NW, NE, SW, SE};

// Initialize X_Drive Instance
X_Drive X_Group(initialMotors);

// Define the controller
controller Controller1 = controller(primary);

// Define the inertial sensor
inertial Inertial = inertial(PORT10);

int main() {
  
  // Calibrate the inertial sensor
  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    Brain.Screen.clearScreen();
    Brain.Screen.print("Inertial Calibrating");
    wait(50, msec);
  }
  // Get the initial angle to calculate the "zero" angle
  double angle_adjust = Inertial.rotation();

  double steering_angle = 0;
  X_Group.set_steeringAngle(steering_angle);
  
  // ToggleB driveButton;
  // driveButton.setValue(false);
  // double cur_angle = Inertial.rotation() - angle_adjust;
  double drive_speed = 0;

  // double drive_percent = NW.get_speed(200*2*M_PI*4.17/2, -45);
  // double max_speed_found = NW.get_max_lin_speed(-45);
  // printf("%7.2f\n", drive_percent);
  // printf("%7.2f\n", max_speed_found);
  // printf("%7.2f\n", 200*2*M_PI*4.17/2);
  // printf("Silly\n");

  while (true){
    

    // steering_angle = Controller1.Axis1.position();
    // drive_speed = Controller1.Axis3.position();
    // printf("%7.2f\n",steering_angle);
    // X_Group.set_speed(drive_speed);
    // // X_Group.set_steeringAngle(steering_angle);
    // X_Group.set_rot_speed(steering_angle/ -10.0);
    // X_Group.update();


    wait(20,msec);
  }
}
