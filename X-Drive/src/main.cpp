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
InertialWrapper Inertial = InertialWrapper(inertial(PORT14));

// EncoderWheel wheel1 = EncoderWheel(rotation(PORT17,false),&Inertial,1, 90,-4.54,-180);
EncoderWheel wheel1 = EncoderWheel(rotation(PORT17,false),&Inertial,1, 180, 1,     90);  // x
EncoderWheel wheel2 = EncoderWheel(rotation(PORT16,false),&Inertial,1,  90, 7.375, 170); // y

// EncoderWheel wheel2 = EncoderWheel(rotation(PORT16,false),&Inertial,1, 90, 1, 90);

std::vector<EncoderWheel> tracking_wheels = {
  wheel1, wheel2
};

OdomWheels Odom(tracking_wheels);

// PathTrace tracer(X_Group,Odom,Inertial);

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
  // wait(2000, msec);
  Brain.Screen.clearScreen();
}



void clamp_open() {
  clamp.set(false);
}
void clamp_close() {
  clamp.set(true);
}
void conveyor_run() {
  conveyor_motor.spin(forward,-100,percent);
}
void conveyor_stop() {
  conveyor_motor.spin(forward,0,percent);
}
void intake_run() {
  intake_motor.spin(forward,100,percent);
}
void intake_stop() {
  intake_motor.spin(forward,0,percent);
}

void x_drive_autonomous(){
  // double starting_x = 96-4;
  // double starting_y = 10.5;
  x_drive_pre_auton();

  double starting_x = 0;
  double starting_y = 0;
  double starting_angle = 0;

  Inertial.setRotation(starting_angle,degrees);
  Odom.update();
  Odom.set_pos(xy_vec(starting_x, starting_y));

  PIDController lin_pid = PIDController(12,0,8);
  // PIDController rot_pid = PIDController(0.9,0,1);
  PIDController rot_pid = PIDController(0.3,0,1);
  PathTraceV3 tracer = PathTraceV3(&X_Group,&Odom,&Inertial,lin_pid,rot_pid);

  conveyor_run();
  wait(100,msec);
  conveyor_stop();

  // tracer.PTurn(xy_vec(100,0),0);
  // tracer.PTurn(xy_vec(100,0),90);
  // tracer.PTurn(xy_vec(100,0),180);
  // tracer.PTurn(xy_vec(100,0),270);
  // tracer.PTurn(xy_vec(100,0),360);
  // tracer.PTurn(xy_vec(100,0),270);
  // tracer.PTurn(xy_vec(100,0),180);
  // tracer.PTurn(xy_vec(100,0),90);
  // tracer.PTurn(xy_vec(100,0),0);
  // tracer.PDrive(xy_vec( 0, 0), 0.2);
  tracer.PDrive(xy_vec(10, 0), 0.2);
  wait(2000,msec);
  tracer.PDrive(xy_vec(10,10), 0.2, -90);
  wait(2000,msec);
  tracer.PDrive(xy_vec( 0,10), 0.2, 180);
  wait(2000,msec);
  tracer.PDrive(xy_vec( 0, 0), 0.2, 90);
  // tracer.PDrive(xy_vec(-10,-10), 0.2, -90);

  // while (true) {

  //   Odom.update();
  //   std::cout << Odom.get_pos().x << "\n";
  //   std::cout << Odom.get_pos().y << "\n\n\n\n\n";
  //   wait(200,msec);
  // }

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
    double max_converyor_speed = 100;
    if (conveyor.getValue(0) || conveyor.getValue(1)){
      if(conveyor.getValue(0)) { 
        conveyor_motor.spin(forward,max_converyor_speed,percent);
        intake_motor.spin(forward,100,percent);}

      else {
        conveyor_motor.spin(forward,-max_converyor_speed,percent);
        intake_motor.spin(forward,-100,percent);
      }
    } else {
      conveyor_motor.spin(forward,0,percent);
      intake_motor.spin(forward,0,percent);
    }

    clamp_toggle.update(Controller1.ButtonY.pressing());
    clamp.set(clamp_toggle.getValue());

    // Odom.update();

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



class nameHolder{
public:
  int name_val1 = 0;
  nameHolder (int name) : name_val1(name) {}

  void setval(int new_name){
    name_val1 = new_name;
  }
};

class changeName {
  public:
  nameHolder* name_obj;
  changeName(nameHolder* new_name) : name_obj(new_name) {}

  void changeto(int new_val){
    name_obj->setval(new_val);
  }
};


int X_Drive_Main(){

  nameHolder name_obj = nameHolder(100);

  changeName change_obj = changeName(&name_obj);

  name_obj.setval(1);

  std::cout << name_obj.name_val1 << "\n";

  change_obj.changeto(2);

  std::cout << name_obj.name_val1 << "\n";




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
