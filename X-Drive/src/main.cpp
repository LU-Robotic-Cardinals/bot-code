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
#include "alamo.h"

using namespace vex;

using signature = vision::signature;
using code = vision::code;

timer Timer;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

int X_Drive_Main(){

  std::vector<vex::motor> initialMotors = {
    motor(PORT19, ratio18_1, false),  // NW Top
    // motor(PORT20, ratio18_1, false),  // NW Dow
    // motor(PORT2,  ratio18_1, false),  // NE Top
    // motor(PORT1,  ratio18_1, false),  // NE Dow
    // motor(PORT11, ratio18_1, false),  // SE Top
    // motor(PORT12, ratio18_1, false),  // SE Dow
    // motor(PORT10, ratio18_1, false),  // SW Top
    // motor(PORT9,  ratio18_1, false)   // SW Dow
  };


  // Initialize X_Drive Instance
  X_Drive X_Group(x_motor_constructor(initialMotors, 2, 1.625, 10, 200, 0, 5.0/3.0));

  // Define the controller
  controller Controller1 = controller(primary);

  // Define the inertial sensor
  inertial Inertial = inertial(PORT13);


  // Calibrate the inertial sensor
  Inertial.calibrate();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Inertial Calibrating");
  while(Inertial.isCalibrating()){
    wait(50, msec);
  }
  Brain.Screen.clearScreen();

  std::vector<EncoderWheel> wheels = {
  // {rotation(PORT6,false),Inertial,1, -90,-4.54,-180},
  {rotation(PORT7,false),Inertial,1, 180,-1.5,-135}
  };

  OdomWheels odom(wheels);

  PathTrace tracer(X_Group,odom,Inertial);

  // std::vector<xy_vec> points = {xy_vec(10,10), xy_vec(650,500), xy_vec(650,-500), xy_vec(10,400),xy_vec(10,300),xy_vec(10,50)};
  // std::vector<xy_vec> points = {xy_vec(0,0), xy_vec(30,50), xy_vec(30,-50), xy_vec(0,30)};
  // BezierCurve bcurve;
  
  // bcurve.setPoints(points);
  // tracer.setCurve(bcurve);

  // std::cout << X_Group.get_max_lin_speed(45) << "\n";

  ExclusiveB conveyor = ExclusiveB(2);
  motor conveyor_motor = motor(PORT9,false);
  motor intake_motor = motor(PORT2,false);
  ToggleB clamp_toggle;

  digital_out clamp = digital_out(Brain.ThreeWirePort.H);

  while (true) {
    conveyor.update(0,Controller1.ButtonL1.pressing());
    conveyor.update(1,Controller1.ButtonL2.pressing());

    // conveyor_motor.spin(forward,0,percent);
    // intake_motor.spin(forward,0,percent);

    // if(Controller1.ButtonL1.pressing()){
    // conveyor_motor.spin(forward,-100,percent);
    // intake_motor.spin(forward,100,percent);
    // }

    // if(Controller1.ButtonL2.pressing()){
    // conveyor_motor.spin(forward,100,percent);
    // intake_motor.spin(forward,-100,percent);
    // }

    if (conveyor.getValue(0) || conveyor.getValue(1)){
      if(conveyor.getValue(0)) { 
        conveyor_motor.spin(forward,100,percent);
        intake_motor.spin(forward,100,percent);}

      else {
        conveyor_motor.spin(forward,-100,percent);
        intake_motor.spin(forward,-100,percent);
      }
    } else {
      conveyor_motor.spin(forward,0,percent);
      intake_motor.spin(forward,0,percent);
    }

    clamp_toggle.update(Controller1.ButtonY.pressing());
    clamp.set(clamp_toggle.getValue());

    // odom.update();

    polar_vec drive_vector = xy_vec(Controller1.Axis3.position(),-Controller1.Axis4.position()).to_polar().add(0,Inertial.rotation());
    double spin = - Controller1.Axis1.position();
    X_Group.set_lin_speed(drive_vector.r/100.0 * X_Group.get_max_lin_speed(drive_vector.theta));
    X_Group.set_steeringAngle(drive_vector.theta);
    X_Group.set_rot_speed(spin/100.0 * X_Group.get_max_rot_speed());

    X_Group.update();
    // wait(10,msec);
    // while (!tracer.update()) {wait(10,msec);}

    wait(1,msec);
  }

}

int main() {
  X_Drive_Main();
}
