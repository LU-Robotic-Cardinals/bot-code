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

competition Competition;

using signature = vision::signature;
using code = vision::code;

timer Timer;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;





  // When looking in direction of brain screen normal
  // with intake on the left
  std::vector<vex::motor> initialMotors = {
    motor(PORT11, ratio18_1, false),  // NW Top
    motor(PORT12, ratio18_1, false),  // NW Dow
    motor(PORT19, ratio18_1, false),  // NE Top
    motor(PORT20, ratio18_1, false),  // NE Dow
    motor(PORT9,  ratio18_1, false),  // SE Top
    motor(PORT10, ratio18_1, false),  // SE Dow
    motor(PORT1,  ratio18_1, false),  // SW Top
    motor(PORT2,  ratio18_1, false)   // SW Dow
  };


  // Initialize X_Drive Instance
  X_Drive X_Group(x_motor_constructor(initialMotors, 2, 1.625, 10, 200, 0, 5.0/3.0));

  // Define the controller
  controller Controller1 = controller(primary);

  // Define the inertial sensor
  inertial Inertial = inertial(PORT14);



  std::vector<EncoderWheel> wheels = {
  {rotation(PORT16,false),Inertial,1, 90,-4.54,-180},
  {rotation(PORT17,false),Inertial,1,   0,-1.5,-135}
  };

  OdomWheels odom(wheels);

  // PathTrace tracer(X_Group,odom,Inertial);

  // std::vector<xy_vec> points = {xy_vec(10,10), xy_vec(650,500), xy_vec(650,-500), xy_vec(10,400),xy_vec(10,300),xy_vec(10,50)};
  // std::vector<xy_vec> points = {xy_vec(0,0), xy_vec(30,50), xy_vec(30,-50), xy_vec(0,30)};
  // BezierCurve bcurve;
  
  // bcurve.setPoints(points);
  // tracer.setCurve(bcurve);

  ExclusiveB conveyor = ExclusiveB(2);
  motor conveyor_motor = motor(PORT18,false);
  motor intake_motor = motor(PORT13,false);
  ToggleB clamp_toggle;

  digital_out clamp = digital_out(Brain.ThreeWirePort.H);



void x_drive_pre_auton(){
  // Calibrate the inertial sensor
  Inertial.calibrate();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Inertial Calibrating");
  while(Inertial.isCalibrating()){
    wait(50, msec);
  }
  Brain.Screen.clearScreen();
}




void x_drive_autonomous(){
  double starting_x = 10;
  double starting_y = 10;
  double starting_angle = 90;


  odom.set_pos(xy_vec(starting_x, starting_y));
  Inertial.setRotation(starting_angle,degrees);


  PIDController lin_pid = PIDController(1,0,0);
  PIDController rot_pid = PIDController(2,0,0);
  PathTraceV3 tracer = PathTraceV3(X_Group,odom,Inertial,lin_pid,rot_pid);

  wait(1000,msec);
  tracer.PTurn(xy_vec(100,0),90);
  wait(1000,msec);
  tracer.PDrive(xy_vec(100,0));

}




void x_drive_usercontrol(){
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

    // polar_vec drive_vector = xy_vec(Controller1.Axis3.position(),-Controller1.Axis4.position()).to_polar().add(0,Inertial.rotation()+90);
    polar_vec drive_vector = xy_vec(Controller1.Axis3.position(),-Controller1.Axis4.position()).to_polar().add(0,90);


    if (Controller1.ButtonDown.pressing())
      drive_vector = polar_vec(95,-90);

    // polar_vec drive_vector = polar_vec(20,90);
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




int X_Drive_Main(){

  Competition.autonomous(x_drive_autonomous);
  Competition.drivercontrol(x_drive_usercontrol);

  // Run the pre-autonomous function.
  x_drive_pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

int main() {
  X_Drive_Main();
}
