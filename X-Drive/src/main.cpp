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
#include "helper.cpp"
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
// {motor(PORT20, ratio18_1, false), wheel_speed, wheel_rad_size,  -45-motor_reference, bot_radius,  1, -1}, // NW Dow

{motor(PORT2,  ratio18_1, false), wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius,  1,  1}, // NE Top
// {motor(PORT1,  ratio18_1, false), wheel_speed, wheel_rad_size,   45-motor_reference, bot_radius, -1, -1}, // NE Dow

{motor(PORT10, ratio18_1, false), wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius,  1,  1}, // SW Top
// {motor(PORT9,  ratio18_1, false), wheel_speed, wheel_rad_size, -135-motor_reference, bot_radius, -1, -1}, // SW Dow

{motor(PORT11, ratio18_1, false), wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius, -1,  1}, //SE Top
// {motor(PORT12, ratio18_1, false), wheel_speed, wheel_rad_size,  135-motor_reference, bot_radius,  1, -1}  //SE Dow
};

// Initialize X_Drive Instance
X_Drive X_Group(initialMotors);

// Define the controller
controller Controller1 = controller(primary);

// Define the inertial sensor
inertial Inertial = inertial(PORT13);

// Define the front distance sensor
distance Distance1 = distance(PORT4);

bool closetonum(double value = 0, double target = 0, double error = 0.001) {
    if (fabs(value - target) < fabs(error)) {
        return true;
    }
    return false;
}

class RollingAverage {
public:
    RollingAverage(int window_size) : window_size_(window_size), sum_(0.0), window_(window_size, 0.0) {}

    void add(double value) {
        sum_ += value - window_[index_];
        window_[index_] = value;
        index_ = (index_ + 1) % window_size_;
    }

    double average() const {
        return sum_ / window_size_;
    }

private:
    int window_size_;
    double sum_;
    std::vector<double> window_;
    int index_ = 0;
};


struct RemoveGravity {
  public:
    RemoveGravity(inertial inertial_sensor, axisType axis, double avg_size) : sensor(inertial_sensor), selected_axis(axis), avg_obj(avg_size){
      // double i_a1 = sensor.acceleration(xaxis);
      // double i_a2 = sensor.acceleration(yaxis);
      // double i_a3 = sensor.acceleration(zaxis);
    }
    double calculate() {
      double a1 = sensor.acceleration(xaxis) ;
      double a2 = sensor.acceleration(yaxis) ;
      double a3 = sensor.acceleration(zaxis) ;

      double a1_2 = pow(a1,2);
      double a2_2 = pow(a2,2);
      double a3_2 = pow(a3,2);

      double net_accell = pow( a1_2 + a2_2 + a3_2,0.5) - 1;

      // double theta = atan(fabs(a1)/fabs(a2)) * getSign(a1) * getSign(a2);
      double theta = atan2(a2, a1) * (180/M_PI);
      double phi = acos(a3);

      // printf("%7.2f",sensor.acceleration(xaxis));

      double axis_value;
      if (selected_axis == xaxis){
        axis_value = sin(phi) * cos(theta) * net_accell;
      } else if (selected_axis == yaxis){
        axis_value = sin(phi) * sin(theta) * net_accell;
      } else if (selected_axis == xaxis){
        axis_value = cos(phi) * net_accell;
      }

      if ((! closetonum(axis_value,0,0.008)) && (axis_value == axis_value)) {
        avg_obj.add(axis_value);
      } else {
        avg_obj.add(0);
      }
    
      // return avg_obj.average();

      return avg_obj.average();
    }
    
  private:
    double i_a1, i_a2, i_a3;
    inertial sensor;
    axisType selected_axis;
    RollingAverage avg_obj;

};

struct InertialPosition {
  public:
    InertialPosition(inertial inertial_sensor, vex::axisType idk_axis) : sensor(inertial_sensor), axis(idk_axis), null_accel(inertial_sensor,idk_axis,40) {
      last_time = Time.time();
      dist = 0;
      last_velocity = 0;
      last_accel = sensor.acceleration(axis);
      base_accel = 0;
    }

    double calculate() {
      // Delta time in seconds
      double conversion = 386.08858267717;
      double dt = (Time.time() - last_time)/10000;
      double velocity = 0;
      double accell = round(null_accel.calculate()*10)/10;

      // Calculations
      // Trapezoid
      // Since this has gravity, that needs to be removed.
      // Some form of 3d pythagorean theorem - 1 needs to happen
      // then for the remainging part, the angle theta and phi needs to be found
      // where theta is on the xy-plane and phi is towoard the z-axis
      // then using vector decomposition to find the magnitude in each direction.
      // This could all be wrapped in a class or a struct and used quite easily in this
      // section of code and others. The class would need a way to set/get the right axis
      // for use in this project. Some form of noise reduction might also be an order.
      double delta_velocity = (accell + last_accel - base_accel) * 0.5 * dt * conversion;
      velocity = last_velocity + delta_velocity; 

      double delta_dist = (velocity + last_velocity) * 0.5 * dt;
      dist = last_dist + delta_dist;
      


      // Execute at end
      last_time = Time.time();
      last_dist = dist;
      last_accel = accell;
      last_velocity = velocity;
      // return dist;
      return velocity;
    }

    double getDist() {
      return dist;
    }

  private:
    double dist;
    double last_dist;
    double last_time;
    double last_accel;
    double last_velocity;
    double base_accel;
    axisType axis;
    inertial sensor;
    timer Time;
    RemoveGravity null_accel;

};

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
  PIDController spinPID(1,0, 40);
  PIDController  linPID(1,0, 20);
  ToggleB ESTOP;
  ToggleB Clamp;
  ESTOP.setValue(false);
  // Clamp.setValue(false);
  SingleB rotateFrontLeft;
  SingleB rotateFrontRight;

  wait(100, msec);

  InertialPosition xdist(Inertial, xaxis); 

  bumper Stop = bumper(Brain.ThreeWirePort.B);

  double current_angle = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.print(X_Group.get_max_rot_speed());
  long last_time = Timer.time();
  while (true) {
    
    ESTOP.update((Controller1.ButtonR1.pressing() || Stop.value()));
    Clamp.update(Controller1.ButtonL1.pressing());
    // Clamp.update(true);

    if (rotateFrontLeft.update(Controller1.ButtonL2.pressing()))
      current_angle += 90;

    if (rotateFrontLeft.update(Controller1.ButtonR2.pressing()))
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
      double speed = Controller1.Axis3.position();

      double spin = spinPID.calculate(Inertial.rotation() - angle_adjust);
      // double dist = linPID.calculate(Distance1.objectDistance(inches)-12);
      // X_Group.set_rot_speed(spin);
      // printf("%7.2f\n",spinPID.values()[0]);
      // if (closetonum(spinPID.values()[0]) && closetonum(spinPID.values()[2])) {
        
        X_Group.set_lin_speed(speed);
        X_Group.set_steeringAngle(current_angle);
      // }
    } else {
     X_Group.set_rot_speed(0); 
     X_Group.set_lin_speed(0);
    }
    X_Group.update();
    wait(1000,msec);
  }
}
