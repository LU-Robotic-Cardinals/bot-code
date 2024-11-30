/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Klein Davis                                               */
/*    Created:      Thu Nov 28 2024                                           */
/*    Description:  Skills code for X-Drive bot                               */
/*                                                                            */
/*    Name:                                                                   */
/*    Date:                                                                   */
/*    Class:                                                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "classes.h"

using namespace vex;

using signature = vision::signature;
using code = vision::code;

timer Timer;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

std::vector<vex::motor> initialMotors = {
  motor(PORT19, ratio18_1, false),  // NW Top
  motor(PORT20, ratio18_1, false),  // NW Dow
  motor(PORT2,  ratio18_1, false),  // NE Top
  motor(PORT1,  ratio18_1, false),  // NE Dow
  motor(PORT11, ratio18_1, false),  // SE Top
  motor(PORT12, ratio18_1, false),  // SE Dow
  motor(PORT10, ratio18_1, false),  // SW Top
  motor(PORT9,  ratio18_1, false)   // SW Dow
};

// Initialize X_Drive Instance
X_Drive X_Group(x_motor_constructor(initialMotors, 2, 1.625, 10, 200, 0, 5.0/3.0));

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

  std::vector<EncoderWheel> wheels = {
  {rotation(PORT6,false),Inertial,1, -90,-4.54,-180},
  {rotation(PORT7,false),Inertial,1, 180,-0.66,-135}
  };

  OdomWheels odom(wheels);

  PathTrace tracer(X_Group,odom,Inertial);
  
  while (true) {
    polar_pos drive_vector = xy_pos(Controller1.Axis3.position(),-Controller1.Axis4.position()).convert_to_polar().add(0,Inertial.rotation());
    double spin = - Controller1.Axis1.position();
    X_Group.set_lin_speed(drive_vector.r/100.0 * X_Group.get_max_lin_speed(drive_vector.theta));
    X_Group.set_steeringAngle(drive_vector.theta);
    X_Group.set_rot_speed(spin/100.0 * X_Group.get_max_rot_speed());

    // X_Group.update();

    // tracer.update();

    wait(10,msec);
  }
}
