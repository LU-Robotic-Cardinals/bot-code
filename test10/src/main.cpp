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

timer Timer;


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

    double get_rot_coef(double rot_speed) {
      if (rot_speed == 0)
        rot_speed = 1;
      return (fabs(rot_speed)/rot_speed) * rot_coef;
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


class X_Drive {
private:
    std::vector<AngledM> motors;
    double max_motor_speed;
    double angle_adjust;
    double linear_speed; //In inches/sec
    double steering_angle; // In degrees
    double rot_speed; // In degrees/sec 

public:
    X_Drive(const std::vector<AngledM>& initialMotors)
        : motors(initialMotors), max_motor_speed(100), angle_adjust(0), linear_speed(0), steering_angle(0), rot_speed(0) {}

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
        std::vector<double> coefs(motors.size());
        double max_coef = 0;

        for (size_t i = 0; i < motors.size(); ++i) {
            coefs[i] = motors[i].get_speed_coef(steering_angle);
            coefs[i] += motors[i].get_rot_coef(rot_speed) * fabs(rot_speed);
            if (fabs(coefs[i]) > max_coef) {
                max_coef = fabs(coefs[i]);
            }
        }

        double scalar = 0;
        if (fabs(max_coef) != 0)
            scalar = (1 / max_coef) * max_motor_speed;

        for (size_t i = 0; i < motors.size(); ++i) {
            coefs[i] = coefs[i] * scalar;
            motors[i].set_speed(coefs[i]);
        }
    }

    void set_speed(double new_speed) {
        max_motor_speed = new_speed;
    }

    void set_rot_speed(double new_speed) {
        rot_speed = new_speed;
    }

    void set_steeringAngle(double new_angle) {
        steering_angle = new_angle;
    }
};

AngledM NW = {motor(PORT17, ratio18_1, false),  -45,   1, -1};
AngledM NE = {motor(PORT18, ratio18_1, false),   45,  -1, -1};
AngledM SW = {motor(PORT20, ratio18_1, false), -135,  -1, -1};
AngledM SE = {motor(PORT19, ratio18_1, false),  135,   1, -1};


std::vector<AngledM> initialMotors = {NW, NE, SW, SE};

X_Drive X_Group(initialMotors);

controller Controller1 = controller(primary);

inertial Inertial = inertial(PORT10);

int main() {

  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    Brain.Screen.clearScreen();
    Brain.Screen.print("Inertial Calibrating");
    wait(50, msec);
  }
  double angle_adjust = Inertial.rotation();

  // double bot_angle = 0;
  double steering_angle = 0;
  X_Group.set_steeringAngle(steering_angle);
  

  // double speed = 40;

  ToggleB driveButton;
  driveButton.setValue(false);
  double find_rot_coef = 10;
  double last_angle = 0;
  double cur_angle = Inertial.rotation() - angle_adjust;
  double drive_speed = 0;

  double omega = 0.01;
  // X_Group.set_rot_speed(omega);

  while (true){
    // double diff_time = 2000;

    // double cur_angle = Inertial.rotation() - angle_adjust;
    // double diff_angle = cur_angle - last_angle;
    
    // if (diff_time > 0){
    //   find_rot_coef = (5.0 * (find_rot_coef/6.0)) + ((omega/diff_angle)/6.0);
    //   // find_rot_coef = ((omega/diff_angle)/6.0);
    // }
    // printf("Here : %7.2f\n",find_rot_coef);
    // printf("%7.2f\n",omega);
    // printf("%7.2f\n",diff_angle);


    // X_Group.update();
    // last_angle = Inertial.rotation() - angle_adjust;
    // wait(2000,msec);


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
