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

timer Timer;


using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

struct AngledM {
    // Struct variable definitions
    vex::motor M;
    double max_wheel_speed; // Rot speed of wheel at 100% power
    double wheel_radius; // Wheel radius
    double wheel_angle; // Angle from arbitrary "front"
    double dist_from_rot_center; // Radius from center of X to middle of wheel contact patch
    double lin_coef; // Special correcting variable for linear motion
    double rot_coef; // Special correcting variable for rotational motion

    double get_lin_speed_coef(double theta) {
      double val = cos( ((theta - wheel_angle)/180.0) * M_PI) * lin_coef;

      // Perhaps not needed
      // if (fabs(val) < (0.01 * fabs(speed_coef)))
      //   val = 0;

      return val;
    }

    // speed is in inches/min
    // Returns motor speed percentage to achive desired linear speed
    // given steering angle, wheel radius, and max motor rotations/min
    double get_lin_speed(double lin_speed, double theta) {
      // Spin coeficient based on relative angles
      double coef = get_lin_speed_coef(theta);
      // Circumfrence... that's it.
      double wheel_circumference = 2 * M_PI * wheel_radius;

      // Number of revolutions per minute it takes for
      // wheel circumference to travel the dist in one minute
      double revs_per_min = lin_speed / wheel_circumference;

      // This calculates with the following units
      // in/min / in/rot * coef = rot/min of wheel
      // rot/min / rot/min = % motor power
      double speed_percent = (revs_per_min * coef) / max_wheel_speed;
      return speed_percent;
    }

    // This calculates the maximum linear speed that the
    // motor can handle
    double get_max_lin_speed(double theta) {
      // Dummy speed of 60 in/min and use given angle
      // Need to have angle because max speed is infinite
      // at certain angles

      // zero_speed is always zero as far as I can tell
      // it is just here in case that changes in the future
      double zero_speed = get_lin_speed(0, theta);
      double one_speed = get_lin_speed(60, theta);
      // Delta speed % / delta speed
      double slope = (one_speed - zero_speed)/60;

      // Since 100% is the max, find how many "slopes" 
      // it takes to max out the motor
      double result = 1.0 / slope;
      return result;
      
    }

    double get_rot_speed_coef(double rot_speed) {
      if (rot_speed == 0)
        rot_speed = 1;
      return (fabs(rot_speed)/rot_speed) * rot_coef;
    }

    // rot_speed is in deg/sec
    double get_rot_speed(double rot_speed) {

      double coef = get_rot_speed_coef(rot_speed);

      // if (rot_speed == 0)
      //   rot_speed = 1;
      double wheel_circumference = 2 * M_PI * wheel_radius;
      
      // Circumfrence of circle traced out
      // by wheel when the bot spins.
      // double arc_circumference = 2 * M_PI * dist_from_rot_center;

      // Length of arc traced out by wheel
      // in one minute
      // deg/sec * rad/deg * dist/rad = dist/sec
      // dist/sec * 60sec/min * 1 min = dist in one minute

      double arc_length = fabs(rot_speed) * (M_PI / 180.0) * dist_from_rot_center;

      // Revs that the wheel has to perform per minute
      // X 60 for sec to minute conversion
      double revs_per_min = arc_length * 60 / wheel_circumference;

      // rot/min
      double speed_percent = (revs_per_min * coef) / max_wheel_speed;

      return speed_percent;
    }

    // This calculates the maximum angular speed that the
    // motor can handle
    double get_max_rot_speed() {
      // Dummy speed of 1 deg/sec
      // Need to have angle because max speed is infinite
      // at certain angles

      // zero_speed is always zero as far as I can tell
      // it is just here in case that changes in the future
      double zero_speed = get_rot_speed(0);
      double one_speed = get_rot_speed(1);
      // Delta speed % / delta speed
      double slope = (one_speed - zero_speed)/1.0;

      // Since 100% is the max, find how many "slopes" 
      // it takes to max out the motor
      double result = 1.0 / slope;
      return fabs(result);
      
    }

    // This just sets the speed of the motor
    // to a percentage value
    void set_speed(double speed) {
      M.spin(forward,speed*100,percent);
    }
};

struct ToggleB {
  // Simple toggle button stuct
  // run setval and update each loop
  bool output = false;
  bool lastUpdatePressed = false;
  
  bool update(bool button) {
    static bool lastUpdatePressed = false;
    if (button && (!lastUpdatePressed))
      output = !output;
    lastUpdatePressed = button;
    return output;
  }
  bool getValue() {
    return output;
  }
  bool setValue(bool setVal) {
    output = setVal;
    return output;
  }
};


class X_Drive {
private:
    std::vector<AngledM> motors;
    double max_motor_speed; // Needs to be set as a percentage 0 <= x <= 1
    double linear_speed; //In inches/min
    double steering_angle; // In degrees
    double rot_speed;

public:
    X_Drive(const std::vector<AngledM>& initialMotors)
        : motors(initialMotors), max_motor_speed(1), linear_speed(0), steering_angle(0), rot_speed(0) {}

    void addMotor(const AngledM& motor) {
        motors.push_back(motor);
    }

    std::vector<AngledM> getMotors() const {
        return motors;
    }

    void removeMotorByIndex(size_t index) {
        if (index < motors.size()) {
            motors.erase(motors.begin() + index);
        }
    }

    void update() {
      // Percent speed for each motor
      std::vector<double> p_speeds(motors.size());

      double max_coef = 0;
      for (size_t i = 0; i < motors.size(); ++i) {
          p_speeds[i]  = motors[i].get_lin_speed(linear_speed,steering_angle);
          p_speeds[i] += motors[i].get_rot_speed(rot_speed);
          if (fabs(p_speeds[i]) > max_coef) {
              max_coef = fabs(p_speeds[i]);
          }
      }

      double overspeed_scalar = 1;              //  \/ \/ \/ \/ \/ This is to make sure the motors don't max out
      if ((fabs(max_coef) > max_motor_speed) || (fabs(max_coef) > 1.0))
          overspeed_scalar = (1.0 / max_coef) * std::min(max_motor_speed, 1.0);

      overspeed_scalar = 1;

      for (size_t i = 0; i < motors.size(); ++i) {
          p_speeds[i] = p_speeds[i] * overspeed_scalar;
          motors[i].set_speed(p_speeds[i]);
          // printf("%7.2f\n",p_speeds[i]);
      }
      // printf("\n");
    }

    void set_max_speed(double new_speed) {
        max_motor_speed = new_speed;
    }

    void set_lin_speed(double new_speed) {
        linear_speed = new_speed;
    }

    void set_steeringAngle(double new_angle) {
        steering_angle = new_angle;
    }
    void set_rot_speed(double new_speed) {
        rot_speed = new_speed;
    }
};

// AngledM NW = {motor(PORT17, ratio18_1, false),  -45,   1, -1};
// AngledM NE = {motor(PORT18, ratio18_1, false),   45,  -1, -1};
// AngledM SW = {motor(PORT20, ratio18_1, false), -135,  -1, -1};
// AngledM SE = {motor(PORT19, ratio18_1, false),  135,   1, -1};

// Wheel radius
// Circumfrence was aprox. 13 + 1/16 inches
// which is 4.157 but 13 + 1/8 is 4.177
double wheel_rad_size = 4.1573 / 2; // In inches
// double bot_radius = 4;
double bot_radius = 12.4417/2;


// For motor speed, it is written for direct-drive of wheels
// i.e. 100 for red, 200 for green, and 600 for blue
// and can be adjusted to account for gear ratios that are 
// external to the motor
// AngledM NW = {motor(PORT17, ratio18_1, false), 200, wheel_rad_size,  -45, bot_radius,  1, -1};
// AngledM NE = {motor(PORT18, ratio18_1, false), 200, wheel_rad_size,   45, bot_radius, -1, -1};
// AngledM SW = {motor(PORT20, ratio18_1, false), 200, wheel_rad_size, -135, bot_radius, -1, -1};
// AngledM SE = {motor(PORT19, ratio18_1, false), 200, wheel_rad_size,  135, bot_radius,  1, -1};

double motor_reference = 0; // Degrees from brain

std::vector<AngledM> initialMotors = {
{motor(PORT19, ratio18_1, false), 200, wheel_rad_size,   45, bot_radius, -1,  1}, // NW Top
{motor(PORT20, ratio18_1, false), 200, wheel_rad_size,   45, bot_radius, 1, -1}, // NW Dow

{motor(PORT2,  ratio18_1, false), 200, wheel_rad_size,  -45, bot_radius,  1,  1}, // NE Top
{motor(PORT1,  ratio18_1, false), 200, wheel_rad_size,  -45, bot_radius, -1, -1}, // NE Dow

{motor(PORT10, ratio18_1, false), 200, wheel_rad_size,  135, bot_radius,  1,  1}, // SW Top
{motor(PORT9,  ratio18_1, false), 200, wheel_rad_size,  135, bot_radius, -1, -1}, // SW Dow

{motor(PORT11, ratio18_1, false), 200, wheel_rad_size, -135, bot_radius, -1,  1}, //SE Top
{motor(PORT12, ratio18_1, false), 200, wheel_rad_size, -135, bot_radius,  1, -1}  //SE Dow
};

// Array of motors to keep track of them
// Not really a need to define the intermediary
// variables (NW, NE, etc.) but doing so for now
// to help with debugging.
// std::vector<AngledM> initialMotors = {NW, NE, SW, SE};

// Initialize X_Drive Instance
X_Drive X_Group(initialMotors);

// Define the controller
controller Controller1 = controller(primary);

// Define the inertial sensor
inertial Inertial = inertial(PORT10);

int main() {
  
  // Calibrate the inertial sensor
  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    Brain.Screen.clearScreen();
    Brain.Screen.print("Inertial Calibrating");
    wait(50, msec);
  }
  // Get the initial angle to calculate the "zero" angle
  double angle_adjust = Inertial.rotation();

  double steering_angle = 0;
  X_Group.set_steeringAngle(steering_angle);
  // X_Group.set_rot_speed(-360);
  X_Group.set_lin_speed(0);
  
  // ToggleB driveButton;
  // driveButton.setValue(false);
  // double cur_angle = Inertial.rotation() - angle_adjust;
  double drive_speed = 0;

  // double drive_percent = NW.get_speed(200*2*M_PI*4.17/2, -45);
  // double max_speed_found = NW.get_max_rot_speed();
  // double max_speed_found = NW.get_rot_speed(24000);
  // printf("%7.2f\n", drive_percent);
  // printf("%7.2f\n", max_speed_found);
  // printf("%7.2f\n", 200*2*M_PI*4.17/2);
  // printf("Silly\n");
  // NW.set_speed(100);

  // printf("%7.2f\n",silly);
  // std::vector<double> speeds_list = {0.1,0.2,0.5,1.0};
  // std::vector<double> speeds_list = {0.5,1.0};
  // double times_list[4] = {0};
  while (true){
    double angle_adjust = Inertial.rotation();
    X_Group.set_rot_speed(angle_adjust*5);
    steering_angle = Controller1.Axis1.position();
    drive_speed = Controller1.Axis3.position();
    printf("%7.2f\n",steering_angle);
    // X_Group.set_speed(drive_speed);
    X_Group.set_lin_speed(drive_speed * 10000/100);
    X_Group.set_rot_speed(steering_angle * -180 / 100);
    X_Group.update();


    wait(20,msec);
  }
  // double avg = 0;
  // int revs = 2;
  // printf("\n\n\n");
  // for (int i = 0; i < speeds_list.size(); i++){
  //   // X_Group.set_rot_speed(speeds_list[i]);
  //   // X_Group.update();
  //   NW.set_speed(speeds_list[i]);
  //   NE.set_speed(speeds_list[i]);
  //   SW.set_speed(speeds_list[i]);
  //   SE.set_speed(speeds_list[i]);
    
  //   // while
  //   wait(500, msec);
  //   double rot_base = Inertial.rotation();
  //   times_list[i] = Timer.time() / 1.0;
  //   while (fabs(Inertial.rotation() - rot_base)<(360*revs)) {
  //     wait(5,msec);
  //   }
  //   times_list[i] = (Timer.time() - times_list[i])/revs;
  //   // 1/ ((200/60)*speeds_list[i]*wheel_rad_size/bot_radius) = times_list[i]/1000
    
  //   // printf("%7.2f\n",times_list[i]/1000);
  //   double print_num = (1000*(200/60)*speeds_list[i]*wheel_rad_size)/(60 * times_list[i]);
  //   printf("%7.5f\n",1/print_num);
  //   // avg += times_list[i]*speeds_list[i]/3;
  // }
  // // printf("Avg: %7.2f\n",avg);
  // // printf("Deviations\n");

  // // for (int i = 0; i < speeds_list.size(); i++){
  // //   printf("%7.2f\n",times_list[i]*speeds_list[i]/avg);
  // // }
  // X_Group.set_rot_speed(0);
  // X_Group.update();
  // // while (true){
  // //   printf("%7.2f\n",Inertial.rotation());
  // // }
}
