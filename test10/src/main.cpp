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

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

struct AngledM {
    vex::motor M;
    double Angle;
    double speed_coef;
    double rot_coef;

    double get_speed_coef(double theta) {
      double val = cos( ((theta - Angle)/180) * M_PI) * speed_coef;
      if (fabs(val) < (0.01 * fabs(speed_coef)))
        val = 0;
      return val;
    }

    double get_rot_coef(double omega) {
      if (omega == 0)
        omega = 1;
      return (fabs(omega)/omega) * rot_coef;
    }

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

// VEXcode device constructors
// AngledM NW = {motor(PORT17, ratio18_1, false),  -45,   1};
// AngledM NE = {motor(PORT18, ratio18_1, false),   45,  -1};
// AngledM SW = {motor(PORT20, ratio18_1, false), -135,  -1};
// AngledM SE = {motor(PORT19, ratio18_1, false),  135,   1};

AngledM NW = {motor(PORT17, ratio18_1, false),  -45,   1, -1};
AngledM NE = {motor(PORT18, ratio18_1, false),   45,  -1, -1};
AngledM SW = {motor(PORT20, ratio18_1, false), -135,  -1, -1};
AngledM SE = {motor(PORT19, ratio18_1, false),  135,   1, -1};

// drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
// motor ClawMotor = motor(PORT3, ratio18_1, false);
// motor ArmMotor = motor(PORT8, ratio18_1, false);

// VEXcode generated functions

controller Controller1 = controller(primary);

inertial Inertial = inertial(PORT10);

int main() {

  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    Brain.Screen.clearScreen();
    Brain.Screen.print("Inertial Calibrating");
    wait(50, msec);
  }
  double angle_adjust = Inertial.roll();

  double bot_angle = 0;
  double steering_angle = 0;
  double speed = 40;
  AngledM motors[4] = {NW,NE,SW,SE};

  ToggleB driveButton;
  driveButton.setValue(false);

  while (true){
  // for (int j = 0; j < 4; j++) {
    double coefs[4];
    double max = 0;

    driveButton.update(Controller1.ButtonL1.pressing());

    if (Controller1.ButtonR1.pressing())
      angle_adjust = Inertial.yaw();

    double cur_angle = Inertial.yaw() - angle_adjust;

    // double omega = (bot_angle + cur_angle);
    steering_angle = cur_angle;
    double omega = steering_angle / speed;

    // printf("%7.3f",omega); 

    for (int i = 0; i < 4; i++) {
      coefs[i] = motors[i].get_speed_coef(bot_angle + steering_angle);
      // coefs[i] = 1;
      coefs[i] += motors[i].get_rot_coef(omega) * fabs(omega);
      if (fabs(coefs[i]) > max) {
        max = fabs(coefs[i]);
      }
    }
    // max = 1;

    // if (motors[0].get_speed_coef(0) == 0){
    //   motors[0].set_speed(100);
    // } else {
    //   motors[0].set_speed(0);
    // }

    // Set speed
    double scalar = 0;
    if (fabs(max) != 0)
      scalar = (1 / max) * speed;
    // double scalar = speed;
    for (int i = 0; i < 4; i++) {
      coefs[i] = coefs[i] * scalar;
      if (!driveButton.getValue()) {
        motors[i].set_speed(0);
      } else {
        motors[i].set_speed(coefs[i]);
      }
    }


    // printf("%7.3f",max); 
    // printf("\n%7.3f\n",max);
    // printf("\n%7.3f\n",steering_angle);

    // steering_angle += 1;
    // bot_angle += 1.8;
    wait(10,msec);
  }
}
