#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include "position.h"

using namespace vex;

#ifndef ODOMETRY_H
#define ODOMETRY_H

struct InertialWrapper {
  public:
    inertial Inertial_point;
    // double scaling_factor = -360.0/357.0;
    double scaling_factor = -1;

    InertialWrapper(inertial Inertial) : Inertial_point(Inertial) {}

    inertial* get_object() {
      // This is a copy of the object and not the actual one
      return &Inertial_point;
    }

    void calibrate() {
      Inertial_point.calibrate();
    }

    bool isCalibrating(){
      return Inertial_point.isCalibrating();
    }

    double rotation() {
      return Inertial_point.rotation() * scaling_factor;
    }

    void setRotation(double value, vex::rotationUnits units) {
      Inertial_point.setRotation(value, units);
    }
};

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

  InertialWrapper* Inertial;
  double last_rot = Inertial->rotation(); // Last rotation value in degrees

  double x_delta = 0; // Change in x since in last update
  double y_delta = 0; // Change in y since in last update

  double x_confidence = 0; // Confidence in delta in x direction, 0 to 1
  double y_confidence = 0; // Confidence in delta in y direction, 0 to 1

public:

  EncoderWheel(vex::rotation encoder_obj, InertialWrapper* inertial_pointer, double wheel_radius_size = 1, 
  double wheel_center_angle = 0, double wheel_offset_radius = 0, double wheel_offset_theta = 0): 
  
  Encoder(encoder_obj), Inertial(inertial_pointer), wheel_radius(wheel_radius_size),
  wheel_angle(wheel_center_angle), offset_radius(wheel_offset_radius), offset_angle(wheel_offset_theta) {}

  void update() {
    // It is assumed that this update function is called sufficiently
    // often so that the math works correctly. IMO, at least 30 Hz
    // depending on anticiated bot speed

    // Current bot rotation
    double current_rot = Inertial->rotation(); // In degrees
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

    // Vector decomposition along cartesian axies of the field
    // These values should be accompanied by their respective
    // confidence values for acurate odometry.
    x_delta = delta_vec.x;
    y_delta = delta_vec.y;

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
  
  xy_vec get_delta_dist() {
    return xy_vec(x_delta,y_delta);
  };

  xy_vec get_delta_confidence() {
    return xy_vec(x_confidence,y_confidence);
  };
};




class OdomWheels {
private:
  std::vector<EncoderWheel> tracking_wheels;
  double x_dist = 0;
  double y_dist = 0;
public:
  OdomWheels(const std::vector<EncoderWheel> initialWheels)
    : tracking_wheels(initialWheels) {}

  void addWheel(const EncoderWheel wheel) {
    tracking_wheels.push_back(wheel);
  }

  std::vector<EncoderWheel> getWheels() const {
    return tracking_wheels;
  }

  void removeWheelByIndex(size_t index) {
    if (index < tracking_wheels.size()) {
      tracking_wheels.erase(tracking_wheels.begin() + index);
    }
  }

  void update() {
    // std::vector<double> x_dist(tracking_wheels.size());
    // std::vector<double> y_dist(tracking_wheels.size());

    std::vector<double> x_conf_sq(tracking_wheels.size());
    std::vector<double> y_conf_sq(tracking_wheels.size());

    double total_x_conf_sq = 0;
    double total_y_conf_sq = 0;
    // Itterate through tracking_wheels
    for (size_t i = 0; i < tracking_wheels.size(); i++) {

      // Update wheel
      tracking_wheels[i].update();

      x_conf_sq[i] = pow(  fabs(tracking_wheels[i].get_delta_confidence().x)  ,2);
      y_conf_sq[i] = pow(  fabs(tracking_wheels[i].get_delta_confidence().y)  ,2);

      // Find conf totals
      total_x_conf_sq += x_conf_sq[i];
      total_y_conf_sq += y_conf_sq[i];
    }

    double delta_x = 0;
    double delta_y = 0;
    for (size_t i = 0; i < tracking_wheels.size(); i++) {
      delta_x += tracking_wheels[i].get_delta_dist().x * x_conf_sq[i] / total_x_conf_sq;
      delta_y += tracking_wheels[i].get_delta_dist().y * y_conf_sq[i] / total_y_conf_sq;
    }
    
    x_dist += delta_x;
    y_dist += delta_y;

    // std::cout << x_dist << "\n";
    // std::cout << y_dist << "\n\n\n\n";

    // std::cout << x_dist << "\n";
    
    // std::cout << "\n\n\n";
    // std::cout << "Total x: " << total_x_conf_sq << "\nTotal y: " << total_y_conf_sq << "\n";
    // std::cout << "X: " << x_dist << "\nY:" << y_dist << "\n\n\n";

    
  }

  xy_vec get_pos(){
    return xy_vec(x_dist,y_dist);
  }

  xy_vec set_pos(xy_vec pos){
    x_dist = pos.x;
    y_dist = pos.y;
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

#endif // ODOMETRY_H