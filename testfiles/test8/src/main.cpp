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

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftDriveSmart = motor(PORT1, ratio18_1, false);
motor RightDriveSmart = motor(PORT2, ratio18_1, true);
// drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
// motor ClawMotor = motor(PORT3, ratio18_1, false);
// motor ArmMotor = motor(PORT8, ratio18_1, false);

// VEXcode generated functions

controller Controller1 = controller(primary);


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  
}
// digital_out dig1 = digital_out( Brain.ThreeWirePort.A );

double getVirtualPosition() {
    static double virtualPosition = 0;
    virtualPosition += 1; // Increment the virtual position
    return virtualPosition;
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // while(true) {
  //   // Get the position of the left joystick (Axis3)
  //   int leftJoystick = Controller1.Axis4.position();

  //   // Set the motor velocity based on the joystick position
  //   LeftDriveSmart.spin(forward, leftJoystick, percent);

  //   // Small delay to prevent wasted resources
  //   vex::task::sleep(20);

  //   // turns cylinder on for one second
  //   if (Controller1.ButtonL1.pressing())
  //     dig1.set(false);
  //   else
  //     dig1.set(true);
  //   // wait(10, seconds);
  // }



  // PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

  // double val = 20;
  // for (int i = 0; i < 100; i++) {
  //     double inc = pid.calculate(0, val);
  //     printf("val:% 7.3f inc:% 7.3f\n", val, inc);
  //     val += inc;
  // }
  
  // Do virtual spring
  // if (controller.button.pressing()) {motor.spinto(desiredposition, degrees);} 


  // double speeds[2] = {0.0, 0.0};  // Initialize speeds array with actual values
  // double diff[2] = {0.0, 0.0};    // Initialize diff array with actual values
  // int positions[2] = {0, 0};      // Initialize positions array with actual values
  // while (true){
    
  //   int leftJoystick = Controller1.Axis3.position();
  //   int rightJoystick = Controller1.Axis2.position();
  //   positions[0] = leftJoystick;
  //   positions[1] = rightJoystick;
  //   int len = (sizeof(speeds)/sizeof(*speeds));
  //   // int len = 2;
  //   if (!Controller1.ButtonL1.pressing()){
  //     for (int i = 0; i < len; i++) {
  //       int difference = speeds[i] - positions[i];
  //       speeds[i] -= 0.01 * std::pow(difference,1);
  //       if (speeds[i] > 100)
  //         speeds[i] = 100;
  //       else if (speeds[i] < -100)
  //         speeds[i] = -100;
  //     }
  //   } else {
  //     for (int i = 0; i < len; i++) {
  //       speeds[i] = positions[i];
  //     }
  //   }
  //   // speeds[0] += 0.5 * (speeds[0] - positions[0]);
  //   // speeds[1] += 0.5 * (speeds[1] - positions[1]);
  //   LeftDriveSmart.spin(forward, speeds[0], percent);
  //   RightDriveSmart.spin(forward, speeds[1], percent);
  //   vex::task::sleep(20);

  // }

  const double kP = 0.5; // Proportional gain
  bool updateVirtualPosition = true; // Flag to control virtual position update

  while(true) {
    // Get the real and virtual positions
    double realPosition = LeftDriveSmart.position(degrees);
    double virtualPosition;
    if (updateVirtualPosition)
      virtualPosition = getVirtualPosition();

    // Calculate the difference
    double positionDifference = virtualPosition - realPosition;

    // Calculate the force (power) to apply
    double motorPower = kP * positionDifference;

    // Check if motor power reaches 100%
    if (motorPower >= 100) {
      motorPower = 100;
      updateVirtualPosition = false; // Stop updating virtual position
    } else if (motorPower <= -100) {
      motorPower = -100;
      updateVirtualPosition = false; // Stop updating virtual position
    } else {
      updateVirtualPosition = true; // Continue updating virtual position
    }

    // Apply the force to the motor
    LeftDriveSmart.spin(forward, motorPower, percent);

    // Print the positions for debugging
    // Brain.Screen.printAt(10, 40, "Real Position: %.2f", realPosition);
    // Brain.Screen.printAt(10, 60, "Virtual Position: %.2f", virtualPosition);
    // Brain.Screen.printAt(10, 80, "Motor Power: %.2f", motorPower);

    // Wait for a short duration
    wait(20, msec);
  }   
}
