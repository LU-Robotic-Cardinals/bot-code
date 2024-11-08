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
#include "classes.cpp"
// #include "helper.cpp"
#include <iostream>

timer Timer;

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

pneumatics Clamp_Actuator = pneumatics(Brain.ThreeWirePort.A);

// Wheel radius
// Circumfrence was aprox. 13 + 1/16 inches
// which is 4.157 but 13 + 1/8 is 4.177
double wheel_rad_size = 3.25 / 2.0; // In inches
// double bot_radius = 4;
double bot_radius = 20/2.0;

double motor_reference = 0; // Degrees from brain
double wheel_speed = 200 * 5 / 3;

std::vector<AngledM> initialMotors = {
{motor(PORT19, ratio18_1, false), wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius, -1,  1}, // NW Top
{motor(PORT20, ratio18_1, false), wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius,  1, -1}, // NW Dow

{motor(PORT2,  ratio18_1, false), wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius,  1,  1}, // NE Top
{motor(PORT1,  ratio18_1, false), wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius, -1, -1}, // NE Dow

{motor(PORT10, ratio18_1, false), wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius,  1,  1}, // SW Top
{motor(PORT9,  ratio18_1, false), wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius, -1, -1}, // SW Dow

{motor(PORT11, ratio18_1, false), wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius, -1,  1}, //SE Top
{motor(PORT12, ratio18_1, false), wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius,  1, -1}  //SE Dow
};

// Initialize X_Drive Instance
X_Drive X_Group(initialMotors);

// Define the controller
controller Controller1 = controller(primary);

// Define the inertial sensor
inertial Inertial = inertial(PORT13);

// Define the front distance sensor
distance Distance1 = distance(PORT4);

int main() {
  
  // Calibrate the inertial sensor
  Inertial.calibrate();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Inertial Calibrating");
  while(Inertial.isCalibrating()){
    wait(50, msec);
  }
  
  Brain.Screen.clearScreen();
  // Get the initial angle to calculate the "zero" angle
  double angle_adjust = Inertial.rotation();
  
  // ToggleB driveButton;
  // driveButton.setValue(false);
  // double cur_angle = Inertial.rotation() - angle_adjust;
  PIDController spinPID(3,0, 4);
  PIDController  linPID(2,0, 2);
  ToggleB ESTOP;
  ToggleB Clamp;
  ESTOP.setValue(true);
  // Clamp.setValue(false);
  SingleB rotateFrontLeft;
  SingleB rotateFrontRight;

  wait(100, msec);

  InertialPosition xdist(Inertial, xaxis); 

  bumper Stop = bumper(Brain.ThreeWirePort.B);

  double current_angle = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.print(X_Group.get_max_rot_speed());
  
  while (true) {
    long last_time = Timer.time();

    ESTOP.update((Controller1.ButtonR1.pressing() || Stop.value()));
    Clamp.update(Controller1.ButtonL1.pressing());
    // Clamp.update(true);

    if (rotateFrontLeft.update(Controller1.ButtonL2.pressing()))
      current_angle += 90;

    if (rotateFrontRight.update(Controller1.ButtonR2.pressing()))
      current_angle -= 90;
    
    // printf("%7.2f\n",X_Group.get_max_rot_speed());
    Clamp_Actuator.set(Clamp.getValue());
    // printf("%7.2f\n",(Timer.time() - last_time)/10000.0);
    // last_time = Timer.time();
    if(ESTOP.getValue()) {
      // Set all speeds to zero and they can be overwritten
      X_Group.set_rot_speed(0);
      X_Group.set_lin_speed(0);    
      // printf("Pitch: %7.2f   Roll: %7.2f   Yaw: %7.2f \n",Inertial.pitch(),Inertial.roll(),Inertial.yaw());
      double speed1 = Controller1.Axis3.position();
      double speed2 = Controller1.Axis4.position();
      double angle = atan2(speed1,speed2)/M_PI*180.0;
      double spin = - Controller1.Axis1.position()/100.0;

      double abs_speed = pow(pow(speed1,2)+pow(speed2,2),0.5);

      // double spin = spinPID.calculate(Inertial.rotation() - angle_adjust);
      // double dist = linPID.calculate(Distance1.objectDistance(inches)-12);
      // X_Group.set_rot_speed(spin*1);
      // printf("%7.2f\n",spinPID.values()[0]);
      // if (closetonum(spinPID.values()[0]) && closetonum(spinPID.values()[2])) {
        
        X_Group.set_rot_speed(spin*X_Group.get_max_rot_speed());
        X_Group.set_lin_speed(abs_speed * 100);
        X_Group.set_steeringAngle(Inertial.rotation() - angle_adjust + angle + current_angle);
        // std::cout << "Angle : " << angle << "\n";
        // std::cout << "Speed : " << abs_speed << "\n";
        std::cout << initialMotors[0].motor.position(degrees);
      // }
    } else {
     X_Group.set_rot_speed(0); 
     X_Group.set_lin_speed(0);
    }
    X_Group.update();

    // std::cout << Timer.time() - last_time << "\n";
    wait(20,msec);

  }
}
