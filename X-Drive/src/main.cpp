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

double wheel_rad_size = 1.625; // In inches
double bot_radius = 10;

double motor_reference = 0; // Degrees from brain
double motor_speed = 200;
double wheel_speed = 200 * 5 / 3;

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

  ToggleB ESTOP;
  ESTOP.setValue(true);

  std::vector<EncoderWheel> wheels = {
  {rotation(PORT6,false),Inertial,1, -90,-4.54,-180},
  {rotation(PORT7,false),Inertial,1, 180,-0.66,-135}
  };

  OdomWheels odom(wheels);

  PathTrace pather(X_Group,odom,Inertial);
  
  while (true) {
    odom.update();
    // Update ESTOP and Clamp buttons with physical button states
    ESTOP.update((Controller1.ButtonR1.pressing()));
    if(ESTOP.getValue()) {      
      pather.update();
    }
    wait(10,msec);
  }
}
