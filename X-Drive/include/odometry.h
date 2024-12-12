#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include "position.h"

using namespace vex;

#ifndef ODOMETRY_H
#define ODOMETRY_H

struct EncoderWheel {
private:
  vex::rotation Encoder;
  double last_encoder = Encoder.position(degrees);
  double wheel_radius; // Wheel radius
  double wheel_angle; // Angle of wheel from arbitrary "front"
  // Right handed world with z-axis normal to floor
  // double wheel_x_offset; // When facing "front", positive x is forward, angle 0 
  // double wheel_y_offset; // When facing "front", positive y is left,    angle 90

  double offset_angle; // Angle of contact patch from arbitrary "front"
  double offset_radius; // Radius from center of rotation

  vex::inertial Inertial;
  double last_rot = Inertial.rotation(); // Last rotation value in degrees
  
  timer Time; // Time object
  double last_time = Time.time(); // Last time value

  double x_delta = 0; // Change in x since in last update
  double y_delta = 0; // Change in y since in last update
  double x_pos = 0; // Current position in x-axis
  double y_pos = 0; // Current position in y-axis

  double x_confidence = 0;
  double y_confidence = 0;

public:

  EncoderWheel(vex::rotation encoder_obj, vex::inertial inertial_obj, double wheel_radius_size = 1, 
  double wheel_center_angle = 0, double wheel_offset_radius = 0, double wheel_offset_theta = 0): 
  
  Encoder(encoder_obj), Inertial(inertial_obj), wheel_radius(wheel_radius_size),
  wheel_angle(wheel_center_angle), offset_radius(wheel_offset_radius), offset_angle(wheel_offset_theta) {}

  void update() {
    // It is assumed that this update function is called sufficiently
    // often so that the math works correctly. IMO, at least 30 Hz
    // depending on anticiated bot speed

    // Current bot rotation
    double current_rot = Inertial.rotation(); // In degrees
    // Delta rotations of bot since last update
    double delta_rot = current_rot - last_rot; // In degrees
    // Average facing direction of bot since last update
    // Since update delay is non-zero, the bot spans an angle
    // from last update time. This value is prefered over
    // current_rot for all direction-based calculations
    double avg_rot = (current_rot + last_rot)/2; // In degrees

    double current_encoder = Encoder.position(degrees);
    // Change in encoder position, in rotations
    double delta_encoder = (current_encoder - last_encoder) / 360.0;
    
    // Change in distance detected by encoder
    // Using wheel circumfrence and delta encoder
    double delta_dist = delta_encoder * 2.0 * M_PI * wheel_radius;

    // Vector discribing the change in location of the center of the bot
    // "center" is defined as the location the wheen is in reference to
    // Both of these vectors are relative to the "world" and not the bot
    xy_vec delta_vec = (  polar_vec(delta_dist,wheel_angle + avg_rot) + (  polar_vec(offset_radius,current_rot) - polar_vec(offset_radius,last_rot)  )  ).to_xy();
    xy_vec confidence_vec = polar_vec(1,wheel_angle + avg_rot).to_xy();

    // "Confidence" essentially means accuracy. At certain angles,
    // the odom wheel might be nearly aligned with an axis but not quite.
    // It will still predict a motion in the axis it is not aligned with,
    // but this value can have a tremendous ammount of error in it, since
    // friction might not be able to overcome rotational friction on the axle.
    x_confidence = fabs(confidence_vec.x);
    y_confidence = fabs(confidence_vec.y);

    // Vector decomposition along cartesian axies
    // These values should be accompanied by their respective
    // confidence values for acurate odometry.
    x_delta = delta_vec.x;
    y_delta = delta_vec.y;
    
    // Calculate new dist
    // These values should not be trusted
    // Might remove them in the future but no need at the moment
    // x_pos += x_delta;
    // y_pos += y_delta;

    // If purely spinning and not moving in any direction,
    // the following code will try to predict the correct distance from
    // center of rotation given angles are correct. 
    // This should be made into a standalone function for use elsewhere in the code.

    // Not finished, need to account for the wheel being at an angle relative to bot
    // with angle_coef
    // double calculated_radius = (delta_encoder / angle_coef * wheel_circumference) / ( (delta_rot/360.0) * 2 * M_PI);
    // d / rot = r
    // double calculated_radius = (delta_encoder * 360.0 * 4 * wheel_radius) / (delta_rot * angle_coef);
    // std::cout << calculated_radius << "\n";

    last_encoder = current_encoder;
    last_rot = current_rot;
  };

  
  void update_old() {
    // It is assumed that this update function is called sufficiently
    // often so that the math works correctly. IMO, at least 30 Hz
    // depending on anticiated bot speed


    // double current_time = Time.time();
    // double dt = current_time - last_time;

    // Current bot rotation
    double current_rot = Inertial.rotation(); // In degrees
    // Delta rotations of bot since last update
    double delta_rot = current_rot - last_rot; // In degrees
    // Average facing direction of bot since last update
    // Since update delay is non-zero, the bot spans an angle
    // from last update time. This value is prefered over
    // current_rot for all direction-based calculations
    double avg_rot = (current_rot + last_rot)/2; // In degrees

    double current_encoder = Encoder.position(degrees);
    // Change in encoder position, in rotations
    double delta_encoder = (current_encoder - last_encoder) / 360.0;

    // Circumference of the encoder wheel, in inches
    double wheel_circumference = 2.0 * M_PI * wheel_radius;

    // Length of the path that the encoder wheel traced out, in inches
    double path_circumference = offset_radius * (delta_rot / 360) * 2 * M_PI;

    // std::cout << (offset_angle) << "\n";

    // Percentage of path_circumference that the encoder will incur, -1 to 1
    double angle_coef = cos(  (wheel_angle  +  90.0  -  offset_angle)   / 180.0*M_PI);

    // The number of incorrect rotations the encoder incurred since last update
    // due to the bot spinning
    double wrong_rots_from_spinning = - (path_circumference * angle_coef) / wheel_circumference;
    
    // Change in distance detected by encoder
    double delta_dist = (delta_encoder - wrong_rots_from_spinning) * wheel_circumference;

    // "Confidence" essentially means accuracy. At certain angles,
    // the odom wheel might be nearly aligned with an axis but not quite.
    // It will still predict a motion in the axis it is not aligned with,
    // but this value can have a tremendous ammount of error in it, since
    // friction might not be able to overcome rotational friction on the axle.
    x_confidence = fabs(cos((avg_rot + wheel_angle)/180 * M_PI));
    y_confidence = fabs(sin((avg_rot + wheel_angle) /180 * M_PI));

    // Vector decomposition along cartesian axies
    // These values should be accompanied by their respective
    // confidence values for acurate odometry.
    x_delta = - cos((avg_rot + wheel_angle) /180 * M_PI) * delta_dist;
    y_delta = sin((avg_rot + wheel_angle) /180 * M_PI) * delta_dist;
    
    // Calculate new dist
    // These values should not be trusted
    // Might remove them in the future but no need at the moment
    x_pos += x_delta;
    y_pos += y_delta;

    // If purely spinning and not moving in any direction,
    // the following code will try to predict the correct distance from
    // center of rotation given angles are correct. 
    // This should be made into a standalone function for use elsewhere in the code.

    // Not finished, need to account for the wheel being at an angle relative to bot
    // with angle_coef
    double calculated_radius = (delta_encoder / angle_coef * wheel_circumference) / ( (delta_rot/360.0) * 2 * M_PI);
    // d / rot = r
    // double calculated_radius = (delta_encoder * 360.0 * 4 * wheel_radius) / (delta_rot * angle_coef);
    std::cout << calculated_radius << "\n";

    // last_time = current_time;
    last_encoder = current_encoder;
    last_rot = current_rot;
  };

  
  xy_vec get_delta_dist() {
    return xy_vec(x_delta,y_delta);
  };

  xy_vec get_delta_confidence() {
    return xy_vec(x_confidence,y_confidence);
  };
};




class OdomWheels {
private:
  std::vector<EncoderWheel> wheels;
  double x_dist = 0;
  double y_dist = 0;
public:
  OdomWheels(const std::vector<EncoderWheel> initialWheels)
    : wheels(initialWheels) {}

  void addWheel(const EncoderWheel wheel) {
    wheels.push_back(wheel);
  }

  std::vector<EncoderWheel> getWheels() const {
    return wheels;
  }

  void removeWheelByIndex(size_t index) {
    if (index < wheels.size()) {
      wheels.erase(wheels.begin() + index);
    }
  }

  void update() {
    // std::vector<double> x_dist(wheels.size());
    // std::vector<double> y_dist(wheels.size());

    std::vector<double> x_conf_sq(wheels.size());
    std::vector<double> y_conf_sq(wheels.size());

    double total_x_conf_sq = 0;
    double total_y_conf_sq = 0;
    // Itterate through wheels
    for (size_t i = 0; i < wheels.size(); i++) {

      // Update wheel
      wheels[i].update();

      x_conf_sq[i] = pow(  fabs(wheels[i].get_delta_confidence().x)  ,2);
      y_conf_sq[i] = pow(  fabs(wheels[i].get_delta_confidence().y)  ,2);

      // Find conf totals
      total_x_conf_sq += x_conf_sq[i];
      total_y_conf_sq += y_conf_sq[i];
    }

    double delta_x = 0;
    double delta_y = 0;
    for (size_t i = 0; i < wheels.size(); i++) {
      delta_x += wheels[i].get_delta_dist().x * x_conf_sq[i] / total_x_conf_sq;
      delta_y += wheels[i].get_delta_dist().y * y_conf_sq[i] / total_y_conf_sq;
    }
    
    x_dist += delta_x;
    y_dist += delta_y;

    // std::cout << x_dist << "\n";
    
    // std::cout << "\n\n\n";
    // std::cout << "Total x: " << total_x_conf_sq << "\nTotal y: " << total_y_conf_sq << "\n";
    // std::cout << "X: " << x_dist << "\nY:" << y_dist << "\n\n\n"; 
  }

  xy_vec get_dist(){
    return xy_vec(x_dist,y_dist);
  }
};



struct BezierCurve {
private:
  std::vector<xy_vec> points = {xy_vec(0,0)};
public:

  void setPoints(std::vector<xy_vec> bPoints) {
    points = bPoints;
  }

  xy_vec calPoint(double t) {
    xy_vec resultant(0,0);
    for (int i = 0; i < points.size(); i++){
      double t_pow = pow(t,i) * pow(1-t,points.size() - (i + 1))
        * binoCoef(points.size()-1,i);
      resultant = resultant + points[i].mul(t_pow,t_pow);
    }
    return resultant;
  }

  xy_vec calDer(double t) {
    xy_vec resultant(0,0);
    for (int i = 0; i < points.size(); i++){
      double f1 = pow(t,i);
      double f2 = pow(1-t,points.size() - (i + 1));
      double df1 = i * pow(t,i - 1);
      double df2 = (points.size() - (i + 1)) 
        * pow(1-t,points.size() - (i + 2));

      double t_pow = (df1 * f2 + f1 * df2)
      * binoCoef(points.size(),i+1);
      resultant = resultant + points[i].mul(t_pow,t_pow);
    }
    return resultant;
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
    xy_vec cur_pos = xy_vec(Odom_Obj.get_dist().x,Odom_Obj.get_dist().y);
    
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
      polar_vec drive_vector = xy_vec(positions[index].x - Odom_Obj.get_dist().x,positions[index].y - Odom_Obj.get_dist().y)
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




#endif // ODOMETRY_H