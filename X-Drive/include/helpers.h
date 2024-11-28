#include <cmath>
#include <string>
#include <vector>
#include <iostream>

bool closetonum(double value = 0, double target = 0, double error = 0.001) {
    if (fabs(value - target) < fabs(error)) {
        return true;
    }
    return false;
}

int getSign(double value) {
  if (value == 0) {
    return 1;
  }
  return (fabs(value) / value);
}

class DelayTimer {
  private:
  timer Time;
  double start_time = Time.time();
  double delay_val = 0;
  public:
  void startTimer(double new_delay = 0) {
    if (new_delay != 0)
      delay_val = new_delay;

    start_time = Time.time();
  }
  bool checkTimer() {
    return Time.time() > (start_time + delay_val);
  }
};

// Forward declaration
struct xy_pos;
struct xyz_pos;
struct polar_pos;
struct spherical_pos;

// Actual declaration
struct xy_pos {
public:
  double x;
  double y;

  xy_pos translate(double delta_x, double delta_y) {
    x += delta_x;
    y += delta_y;
    return *this;
  }

  // Overloaded + operator
  xy_pos operator+(const xy_pos& other) const {
      return xy_pos(x + other.x, y + other.y);
  }

  xy_pos(double x_val, double y_val) : x(x_val), y(y_val) {}

  ///////// Conversions

  // Flat ???
  // xyz_pos convert_to_xyz() {
  //   return xyz_pos(x,y,0);
  // }
  polar_pos convert_to_polar() {
    return polar_pos(pow(pow(x,2)+pow(y,2),0.5),atan2(y,x) / M_PI * 360.0);
  }
};
struct xyz_pos {
public:
  double x;
  double y;
  double z;

  xyz_pos translate(double delta_x, double delta_y, double delta_z) {
    x += delta_x;
    y += delta_y;
    z += delta_z;
    return *this;
  }

  xy_pos operator+(const xy_pos& other) const {
      return xy_pos(x + other.x, y + other.y);
  }

  xyz_pos(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}

  ///////// Conversions

  spherical_pos convert_to_spherical() {
    double rho = pow( pow(x,2) + pow(y,2) + pow(z,2) ,0.5);
    double theta = atan2(y,x) / M_PI * 360.0;
    double phi = acos(z / (rho));
    return spherical_pos(rho, theta, phi);
  }
};
struct polar_pos {
public:
  double r;
  double theta;

  polar_pos translate(double delta_r, double delta_theta) {
    r += delta_r;
    theta += delta_theta;
    return *this;
  }

  polar_pos(double r_val, double theta_val) : r(r_val), theta(theta_val) {}

  ///////// Conversions

  // Flat ???
  // spherical_pos convert_to_spherical() {
  //   return spherical_pos(r,theta,0);
  // }
  xy_pos convert_to_xy(){
    double x = r * cos(theta / M_PI * 360.0);
    double y = r * sin(theta / M_PI * 360.0);
    return xy_pos(x,y);
  }
};
struct spherical_pos {
public:
  double rho;
  double theta;
  double phi;

  spherical_pos translate(double delta_rho, double delta_theta, double delta_phi) {
    rho += delta_rho;
    theta += delta_theta;
    phi += delta_phi;
    return *this;
  }

  spherical_pos(double rho_value, double theta_value, double phi_value) : rho(rho_value), theta(theta_value), phi(phi_value) {}

  ///////// Conversions

  xyz_pos convert_to_xyz() {
    double x = rho * sin(phi * M_PI / 360.0) * cos(theta * M_PI / 360.0);
    double y = rho * sin(phi * M_PI / 360.0) * sin(theta * M_PI / 360.0);
    double z = rho * cos(phi * M_PI / 360.0);
    return xyz_pos(x,y,z);
  }
};