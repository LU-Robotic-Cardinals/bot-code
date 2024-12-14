#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include "position.h"
#include "odometry.h"
#include "drive.h"

using namespace vex;

#ifndef PATHING_H
#define PATHING_H


// Need to make new names for the functions
// All blocking? yea probably 
// Overloaded definitions for different drivetrain objs
// Perhaps find max speed directions as "main" directions and "prefer" those
// directions when turning. PTurn_Snap snap to "main" dirs
// somehow label which main I want the drivetrain to use, perhaps
// find which main direction is closest to a certain bot angle and not
// angle to point


// PTurn
// Turn to face a point
// No PID, linear, calculates shortest turn to face
// Curious how it works with x_drive since can face any direction
// Perhaps some xy_vec.to_polar() rotation stuff

// PDrive
// Drive to a point
// Yes PID

// Turn
// Turn to an absolute angle on field
// No PID

// ThrDrive
// Drive through set of points
// Yes PID


class PathTraceV3 {
  private:
    // std::vector<std::vector<int>> positions = {{0,8},{0,16},{0,0}};
    // std::vector<xy_vec> positions = {xy_vec(0,0),xy_vec(0,16)};
    OdomWheels* Odom_Obj;
    X_Drive* X_Group;
    DelayTimer pos_delay;
    InertialWrapper* Inertial;
    // These are not pointers, so new objs are created on initialization of
    // the class.
    PIDController rot_pid;
    PIDController lin_pid;
    double t = 0;
  public:
  PathTraceV3(X_Drive* X_Drive_Group, OdomWheels* Odom, InertialWrapper* Inertial_Sensor, PIDController lin_pid_obj, PIDController rot_pid_obj) : X_Group(X_Drive_Group), Odom_Obj(Odom), Inertial(Inertial_Sensor), lin_pid(lin_pid_obj), rot_pid(rot_pid_obj) {}

  void PTurn(xy_vec point, double offset_angle = 0){
    // Turn to face a point
    // No PID, linear, calculates shortest turn to face
    // Curious how it works with x_drive since can face any direction
    // Perhaps some xy_vec.to_polar() rotation stuff

    // Need to add some non-blocking mode to use in other PathTrace functions

    bool StillMoving = true;
    int msecPassed = 0;
    int stillCounter = 10;

    while (StillMoving){
      Odom_Obj->update();
      double bot_angle = Inertial->rotation();
      
      // The xy position of the bot. 0,0 should be the bottom left corner of the field
      // relative to where the drive team stands.
      xy_vec bot_position = Odom_Obj->get_pos();

      std::cout << "\n" << bot_position.x << "\n";
      std::cout << bot_position.y << "\n\n";

      double angle_to_point = (point - bot_position).to_polar().theta;
      angle_to_point = 0;

      // std::cout << Inertial->rotation() << "\n";
      // std::cout << angle_to_point << "\n\n";
      // int rotation_direction = getRotationDirection( - offset_angle - angle_to_point, fmod(Inertial->rotation(),360));
      int rotation_direction = getRotationDirection( angle_to_point + offset_angle, Inertial->rotation());
      double angle_error = fabs(fmod(Inertial->rotation() - offset_angle - angle_to_point,360.0)) * rotation_direction;

      double turn_speed = - rot_pid.calculate(angle_error*X_Group->get_max_rot_speed()/100);

      X_Group->set_rot_speed(turn_speed);

      // If derivative near zero and has been trying to turn for
      // 150 msec, then the bot is still.
      // If still for a while then break out.
      if(fabs(rot_pid.values()[3]) <= 0.15 && msecPassed > 150) {
        stillCounter ++;
        if (stillCounter > 10)
          StillMoving = false;
      } else {
      stillCounter = 0;
      }
      
      // Update drivetrain
      X_Group->update();

      // Increment counter and wait
      msecPassed += 10;
      wait(10,msec);
    }

    // After breaking out, then stop the rotation
    X_Group->set_rot_speed(0);
    X_Group->update();
  }




  void PDrive(xy_vec point, double speed = 0.95, double offset_angle = 0){
    // Offset angle clarification:
    // It is the equivalent of the bot's 

    // Drive to a point
    // Yes PID
    bool StillMoving = true;
    int msecPassed = 0;
    int stillCounter = 10;



      // The xy position of the bot. 0,0 should be the bottom left corner of the field
      // relative to where the drive team stands.
      xy_vec initial_bot_position = Odom_Obj->get_pos();

    while (StillMoving){
      Odom_Obj->update();
      double bot_angle = Inertial->rotation();

      // The xy position of the bot. 0,0 should be the bottom left corner of the field
      // relative to where the drive team stands.
      xy_vec bot_position = Odom_Obj->get_pos();

      // std::cout << "\n" << bot_position.x << "\n";
      // std::cout << bot_position.y << "\n\n";

      double angle_to_point = (point - initial_bot_position).to_polar().theta;

      // std::cout << Inertial->rotation() << "\n";
      // std::cout << angle_to_point << "\n\n";
      // int rotation_direction = getRotationDirection( - offset_angle - angle_to_point, fmod(Inertial->rotation(),360));
      int rotation_direction = getRotationDirection( angle_to_point + offset_angle, Inertial->rotation());
      double angle_error = fabs(fmod(Inertial->rotation() - offset_angle - angle_to_point,360.0)) * rotation_direction;

      double turn_speed = - rot_pid.calculate(angle_error*X_Group->get_max_rot_speed()/100);

      X_Group->set_rot_speed(turn_speed);

    

      double dist_error = (point - bot_position).to_polar().r;

      double lin_speed = lin_pid.calculate(dist_error);

      double angle_to_point2 = (point - bot_position).to_polar().theta;

      double steering_angle = angle_to_point2-Inertial->rotation();

      // std::cout << steering_angle << "\n";
      X_Group->set_steeringAngle(steering_angle);
      X_Group->set_lin_speed(lin_speed * X_Group->get_max_lin_speed(steering_angle)/100.0 * speed);


      // If derivative near zero and has been trying to turn for
      // 150 msec, then the bot is still.
      // If still for a while then break out.
      std::cout << lin_pid.values()[3] << "\n";
      if(fabs(lin_pid.values()[3]) <= 0.05 && msecPassed > 150) {
        stillCounter ++;
        if (stillCounter > 10)
          StillMoving = false;
      } else
      stillCounter = 0;



      
      // Update drivetrain
      X_Group->update();

      // Increment counter and wait
      msecPassed += 10;
      wait(10,msec);
    }

    // After breaking out, then stop the rotation
    X_Group->set_rot_speed(0);
    X_Group->set_lin_speed(0);
    X_Group->update();
  }

  void Turn(){
    // Turn to an absolute angle on field
    // No PID
  }void ThrDrive(){
    // Drive through set of points
    // Yes PID
  }
};



class PathTrace {
  private:
    // std::vector<std::vector<int>> positions = {{0,8},{0,16},{0,0}};
    // std::vector<xy_vec> positions = {xy_vec(0,0),xy_vec(0,16)};
    OdomWheels Odom_Obj;
    X_Drive X_Group;
    DelayTimer pos_delay;
    inertial Inertial;
    BezierCurve curve;
    double t = 0;
  public:
  PathTrace(X_Drive X_Drive_Group, OdomWheels Odom, inertial Inertial_Sensor) : X_Group(X_Drive_Group), Odom_Obj(Odom), Inertial(Inertial_Sensor) {}

  void setCurve(BezierCurve new_curve) {
    curve = new_curve;
  }
  bool update()
  { 
    X_Group.set_rot_speed(0);
    X_Group.set_lin_speed(0);
    X_Group.set_steeringAngle(0);
    Odom_Obj.update();
    double tolerence = 1;
    double new_t = 0;
    xy_vec cur_pos = xy_vec(Odom_Obj.get_pos().x,Odom_Obj.get_pos().y);
    
    for (new_t = t; (curve.calPoint(new_t) - cur_pos).to_polar().r < tolerence && (t < 1); new_t += 0.0001) { t = new_t; }
    
    // The amount of math that this new library takes care of is incredible
    polar_vec drive_vector = (curve.calPoint(t) - cur_pos).to_polar().add(0,Inertial.rotation()).mul(1/10.0,1);
    
    if (fabs(drive_vector.r) > 0.2)
      drive_vector.r = 0.2;

    X_Group.set_lin_speed(drive_vector.r * X_Group.get_max_lin_speed(drive_vector.theta));
    X_Group.set_steeringAngle(drive_vector.theta);

    X_Group.update();

    return (drive_vector.r < 0.01);
  }
};










class PathTraceV2 {
  private:
    std::vector<xy_vec> positions = {xy_vec(0,0),xy_vec(0,16)};
    BezierCurve curve;
    OdomWheels Odom_Obj;
    X_Drive X_Group;
    DelayTimer pos_delay;
    inertial Inertial;
    int index = 0;
  public:
  PathTraceV2(X_Drive X_Drive_Group, OdomWheels Odom, inertial Inertial_Sensor) : X_Group(X_Drive_Group), Odom_Obj(Odom), Inertial(Inertial_Sensor) {}
  void update()
  { 
    Odom_Obj.update();
    if (pos_delay.checkTimer()) {

      // The amount of math that this new library takes care of is incredible
      polar_vec drive_vector = xy_vec(positions[index].x - Odom_Obj.get_pos().x,positions[index].y - Odom_Obj.get_pos().y)
      .to_polar().add(0,Inertial.rotation()).mul(1/10.0,1);
      
      if (fabs(drive_vector.r) > 0.2)
        drive_vector.r = 0.2;

      X_Group.set_lin_speed(drive_vector.r * X_Group.get_max_lin_speed(drive_vector.theta));
      X_Group.set_steeringAngle(drive_vector.theta);

      if (drive_vector.r < 0.1) {
        pos_delay.startTimer(10);
        index++;
        if (index == positions.size())
          index = 0;
      }
    } else {
      X_Group.set_rot_speed(0);
      X_Group.set_lin_speed(0);
      X_Group.set_steeringAngle(0);
    }
    X_Group.update();
  }
};

#endif // PATHING_H