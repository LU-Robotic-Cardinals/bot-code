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

timer Timer;

using namespace vex;
using signature = vision::signature;
using code = vision::code;

std::vector<motor>

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

int main() {
  while(true) {
    
    wait(20,msec);
    
  }
}
