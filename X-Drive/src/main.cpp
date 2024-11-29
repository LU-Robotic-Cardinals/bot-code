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
#include "classes.h"
// #include "helper.cpp"
#include <iostream>
#include "odometry.h"
#include <position_declarations.h>

using namespace vex;

timer Timer;

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
double motor_speed = 200;
double wheel_speed = 200 * 5 / 3;

double Kp = 1;
double Ki = 0;
double Kd = 0;
double Mp = 1;
double Mi = 0;
double Md = 0;

std::vector<AngledM> initialMotors = {
{motor(PORT19, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius, -1,  1}, // NW Top
{motor(PORT20, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius,  1, -1}, // NW Dow

{motor(PORT2,  ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius,  1,  1}, // NE Top
{motor(PORT1,  ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius, -1, -1}, // NE Dow

{motor(PORT10, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius,  1,  1}, // SW Top
{motor(PORT9,  ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius, -1, -1}, // SW Dow

{motor(PORT11, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius, -1,  1}, //SE Top
{motor(PORT12, ratio18_1, false), motor_speed, wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius,  1, -1}  //SE Dow
};

// Initialize X_Drive Instance
X_Drive X_Group(initialMotors);

// Define the controller
controller Controller1 = controller(primary);

// Define the inertial sensor
inertial Inertial = inertial(PORT13);

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

  ToggleB ESTOP;
  ToggleB Clamp;
  ESTOP.setValue(true);
  Clamp.setValue(false);
  SingleB rotateFrontLeft;
  SingleB rotateFrontRight;

  double current_angle = 0;

  // GenerateBoxes box_generator;
  // ShowBoxes box_shower;
  // std::cout << box_generator.generate(2,2)[0].br_x << "\n";
  // box_shower.show(Brain, box_generator.generate(3,2));  
  // rotation Rot = rotation(PORT6,false);

  std::vector<EncoderWheel> wheels = {
  {rotation(PORT6,false),inertial(PORT13),1, -90,-4.54,-180},
  {rotation(PORT7,false),inertial(PORT13),1, 180,-0.66,-135}
  };

  OdomWheels odom(wheels);

  PathTrace pather(X_Group,odom,Inertial);
  
  while (true) {
    odom.update();
    // Update ESTOP and Clamp buttons with physical button states
    ESTOP.update((Controller1.ButtonR1.pressing()));
    // Clamp.update(Controller1.ButtonL1.pressing());

    // if (rotateFrontLeft.update(Controller1.ButtonL2.pressing()))
    //   current_angle += 90;

    // if (rotateFrontRight.update(Controller1.ButtonR2.pressing()))
    //   current_angle -= 90;
    
    // Clamp_Actuator.set(Clamp.getValue());
    if(ESTOP.getValue()) {      
      


      // double speed1 = Controller1.Axis4.position() / 100.0;
      // double speed2 = -Controller1.Axis3.position() / 100.0;
      // double angle = atan2(speed1,speed2)/M_PI*180.0;
      // // double spin =  0;
      // double spin = - Controller1.Axis1.position()/100.0;

      // double abs_speed = pow(pow(speed1,2)+pow(speed2,2),0.5);
      // double steering = Inertial.rotation() - angle_adjust + angle + current_angle;

      // X_Group.set_rot_speed(spin*X_Group.get_max_rot_speed());
      // X_Group.set_lin_speed(abs_speed * X_Group.get_max_lin_speed(steering));
      // X_Group.set_steeringAngle(steering);
    // } else {
      // X_Group.set_rot_speed(0);
      // X_Group.set_lin_speed(0);
      pather.update();
    }
    // X_Group.update();

    wait(10,msec);
  }
}
