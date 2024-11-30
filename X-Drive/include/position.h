#include <cmath>

#ifndef POSITION_H
#define POSITION_H

// This whole file is shenanigans; there's no getting around it.
// All of the following is just cpp weirdness to get around
// the cpp compiler evaluating from the top of the file downward.
// Intelesense can manage to do it and gives no error, but the compiler
// doesn't like it. So, for future reference and to save the sanity
// of whichever poor soul needs to make changes to this in the future
// here is the explanation:


// The following is a definition of empty position structs so that the
// the compiler has a preliminary struct to reference.
// In the second section these structs are then redefined with their
// correct private variables and empty functions for the compiler to 
// reference going forward. Finally, in the third section, these functions
// are fully defined, now that the compiler can reference the other structs.

// To add a new struct, add its "struct {name};" to the first section,
// then "struct {name} {code};" where the code is the declaration function
// of the struct and the shells of the functions that will be used.
// Finally, reference those shell functions as a function definiton in
// the third section like "{return_type} {name}::{function_name}(foo) {code};"

// The rest of this code is lacking comments and substandard documentation
// but im too busy trying to be done with this and get to using it.

// Notes:
// 1. Already have translate for adding to or subtracting from the values
// Need transform for multiply/divide
// 2. As a goal, all functions should be non-destructive.



///////////////////////////////////////////////////////////////////////////
/////////////////////////      First section      /////////////////////////
///////////////////////////////////////////////////////////////////////////
struct xy_pos;
struct xyz_pos;
struct polar_pos;
struct spherical_pos;




///////////////////////////////////////////////////////////////////////////
/////////////////////////      Second section     /////////////////////////
///////////////////////////////////////////////////////////////////////////


///////////////////////// xy_pos /////////////////////////
struct xy_pos {
public:
  double x;
  double y;

  xy_pos(double x_val, double y_val) : x(x_val), y(y_val) {}



  xy_pos operator+(const xy_pos& other) const;
  xy_pos operator-(const xy_pos& other) const;

  xy_pos add(double delta_x, double delta_y);
  xy_pos mul(double coef_x, double coef_y);

  xyz_pos convert_to_xyz();
  polar_pos convert_to_polar();
  spherical_pos convert_to_spherical();
};




///////////////////////// xyz_pos /////////////////////////
struct xyz_pos {
public:
  double x;
  double y;
  double z;

  xyz_pos(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}



  xyz_pos operator+(const xyz_pos& other) const;
  xyz_pos operator-(const xyz_pos& other) const;

  xyz_pos add(double delta_x, double delta_y, double delta_z);
  xyz_pos mul(double coef_x, double coef_y, double coef_z);

  xy_pos convert_to_xy();
  polar_pos convert_to_polar();
  spherical_pos convert_to_spherical();
};




///////////////////////// polar_pos /////////////////////////
struct polar_pos {
public:
  double r;
  double theta;

  polar_pos(double r_val, double theta_val) : r(r_val), theta(theta_val) {}

  polar_pos operator+(const polar_pos& other) const;
  polar_pos operator-(const polar_pos& other) const;

  polar_pos add(double delta_r, double delta_theta);
  polar_pos mul(double coef_r, double coef_theta);

  xy_pos convert_to_xy();
  xyz_pos convert_to_xyz();
  spherical_pos convert_to_spherical();
};




///////////////////////// spherical_pos /////////////////////////
struct spherical_pos {
public:
  double rho;
  double theta;
  double phi;

  spherical_pos(double rho_value, double theta_value, double phi_value) : rho(rho_value), theta(theta_value), phi(phi_value) {}


  spherical_pos operator+(const spherical_pos& other) const;
  spherical_pos operator-(const spherical_pos& other) const;

  spherical_pos add(double delta_rho, double delta_theta, double delta_phi);
  spherical_pos mul(double coef_rho, double coef_theta, double coef_phi);

  xy_pos convert_to_xy();
  xyz_pos convert_to_xyz();
  polar_pos convert_to_polar();
};











///////////////////////////////////////////////////////////////////////////
/////////////////////////      Third section      /////////////////////////
///////////////////////////////////////////////////////////////////////////


///////////////////////// xy_pos definitions /////////////////////////
xy_pos xy_pos::add(double delta_x, double delta_y) {
  double new_x = x + delta_x;
  double new_y = y + delta_y;
  return xy_pos(x,y);
}
xy_pos xy_pos::mul(double coef_x, double coef_y) {
  double new_x = x * coef_x;
  double new_y = y * coef_y;
  return xy_pos(x,y);
}


xy_pos xy_pos::operator+(const xy_pos& other) const {
  return xy_pos(x + other.x, y + other.y);
}
xy_pos xy_pos::operator-(const xy_pos& other) const {
  return xy_pos(x - other.x, y - other.y);
}


xyz_pos xy_pos::convert_to_xyz() {
  return xyz_pos(x,y,0);
}
polar_pos xy_pos::convert_to_polar() {
  return polar_pos(pow(pow(x,2)+pow(y,2),0.5),atan2(y,x) / M_PI * 180.0);
}
spherical_pos xy_pos::convert_to_spherical() {
  return spherical_pos(pow(pow(x,2)+pow(y,2),0.5),atan2(y,x) / M_PI * 180.0,0);
}



///////////////////////// xyz_pos definitions /////////////////////////

xyz_pos xyz_pos::add(double delta_x, double delta_y, double delta_z) {
  double new_x = x + delta_x;
  double new_y = y + delta_y;
  double new_z = z + delta_z;
  return xyz_pos(new_x,new_y,new_z);
}
xyz_pos xyz_pos::mul(double coef_x, double coef_y, double coef_z) {
  double new_x = x * coef_x;
  double new_y = y * coef_y;
  double new_z = z * coef_z;
  return xyz_pos(new_x,new_y,new_z);
}


xyz_pos xyz_pos::operator+(const xyz_pos& other) const {
  return xyz_pos(x + other.x, y + other.y, z + other.z);
}
xyz_pos xyz_pos::operator-(const xyz_pos& other) const {
  return xyz_pos(x - other.x, y - other.y, z - other.z);
}


xy_pos xyz_pos::convert_to_xy() {
  return xy_pos(x,y);
}
polar_pos xyz_pos::convert_to_polar() {
  return polar_pos(pow(pow(x,2)+pow(y,2),0.5),atan2(y,x) / M_PI * 180.0);
}
spherical_pos xyz_pos::convert_to_spherical() {
  double rho = pow( pow(x,2) + pow(y,2) + pow(z,2) ,0.5);
  double theta = atan2(y,x) / M_PI * 180.0;
  double phi = acos(z / (rho));
  return spherical_pos(rho, theta, phi);
}




///////////////////////// polar_pos definitions /////////////////////////

polar_pos polar_pos::add(double delta_r, double delta_theta) {
  double new_r = r + delta_r;
  double new_theta = theta + delta_theta;
  return polar_pos(new_r,new_theta);
}
polar_pos polar_pos::mul(double coef_r, double coef_theta) {
  double new_r = r * coef_r;
  double new_theta = theta * coef_theta;
  return polar_pos(new_r,new_theta);
}


polar_pos polar_pos::operator+(const polar_pos& other) const {
  double x = r * cos(theta / M_PI * 180.0);
  double y = r * sin(theta / M_PI * 180.0);
  double other_x = other.r * cos(other.theta / M_PI * 180.0);
  double other_y = other.r * sin(other.theta / M_PI * 180.0);

  double new_x = x + other_x;
  double new_y = y + other_y;

  return polar_pos(pow(pow(new_x,2)+pow(new_y,2),0.5),atan2(new_y,new_x) / M_PI * 180.0);
}
polar_pos polar_pos::operator-(const polar_pos& other) const {
  double x = r * cos(theta / M_PI * 180.0);
  double y = r * sin(theta / M_PI * 180.0);
  double other_x = other.r * cos(other.theta / M_PI * 180.0);
  double other_y = other.r * sin(other.theta / M_PI * 180.0);

  double new_x = x - other_x;
  double new_y = y - other_y;

  return polar_pos(pow(pow(new_x,2)+pow(new_y,2),0.5),atan2(new_y,new_x) / M_PI * 180.0);
}


spherical_pos polar_pos::convert_to_spherical() {
  return spherical_pos(r,theta,0);
}
xy_pos polar_pos::convert_to_xy(){
  double x = r * cos(theta / M_PI * 180.0);
  double y = r * sin(theta / M_PI * 180.0);
  return xy_pos(x,y);
}
xyz_pos polar_pos::convert_to_xyz(){
  double x = r * cos(theta / M_PI * 180.0);
  double y = r * sin(theta / M_PI * 180.0);
  return xyz_pos(x,y,0);
}




///////////////////////// spherical_pos definitions /////////////////////////
spherical_pos spherical_pos::add(double delta_rho, double delta_theta, double delta_phi) {
  double new_rho = rho + delta_rho;
  double new_theta = theta + delta_theta;
  double new_phi = phi + delta_phi;
  return spherical_pos(new_rho,new_theta,new_phi);
}
spherical_pos spherical_pos::mul(double coef_rho, double coef_theta, double coef_phi) {
  double new_rho = rho * coef_rho;
  double new_theta = theta * coef_theta;
  double new_phi = phi * coef_phi;
  return spherical_pos(new_rho,new_theta,new_phi);
}


spherical_pos spherical_pos::operator+(const spherical_pos& other) const {
  double x = rho * sin(phi * M_PI / 180.0) * cos(theta * M_PI / 180.0);
  double y = rho * sin(phi * M_PI / 180.0) * sin(theta * M_PI / 180.0);
  double z = rho * cos(phi * M_PI / 180.0);
  double other_x = other.rho * sin(other.phi * M_PI / 180.0) * cos(other.theta * M_PI / 180.0);
  double other_y = other.rho * sin(other.phi * M_PI / 180.0) * sin(other.theta * M_PI / 180.0);
  double other_z = other.rho * cos(other.phi * M_PI / 180.0);

  double new_x = x + other_x;
  double new_y = y + other_y;
  double new_z = y + other_z;

  double new_rho = pow( pow(new_x,2) + pow(new_y,2) + pow(new_z,2) ,0.5);
  double new_theta = atan2(new_y,new_x) / M_PI * 180.0;
  double new_phi = acos(new_z / (new_rho));
  return spherical_pos(new_rho, new_theta, new_phi);
}
spherical_pos spherical_pos::operator-(const spherical_pos& other) const {
  double x = rho * sin(phi * M_PI / 180.0) * cos(theta * M_PI / 180.0);
  double y = rho * sin(phi * M_PI / 180.0) * sin(theta * M_PI / 180.0);
  double z = rho * cos(phi * M_PI / 180.0);
  double other_x = other.rho * sin(other.phi * M_PI / 180.0) * cos(other.theta * M_PI / 180.0);
  double other_y = other.rho * sin(other.phi * M_PI / 180.0) * sin(other.theta * M_PI / 180.0);
  double other_z = other.rho * cos(other.phi * M_PI / 180.0);

  double new_x = x - other_x;
  double new_y = y - other_y;
  double new_z = y - other_z;

  double new_rho = pow( pow(new_x,2) + pow(new_y,2) + pow(new_z,2) ,0.5);
  double new_theta = atan2(new_y,new_x) / M_PI * 180.0;
  double new_phi = acos(new_z / (new_rho));
  return spherical_pos(new_rho, new_theta, new_phi);
}


xy_pos spherical_pos::convert_to_xy() {
  double x = rho * sin(phi * M_PI / 180.0) * cos(theta * M_PI / 180.0);
  double y = rho * sin(phi * M_PI / 180.0) * sin(theta * M_PI / 180.0);
  return xy_pos(x,y);
}
xyz_pos spherical_pos::convert_to_xyz() {
  double x = rho * sin(phi * M_PI / 180.0) * cos(theta * M_PI / 180.0);
  double y = rho * sin(phi * M_PI / 180.0) * sin(theta * M_PI / 180.0);
  double z = rho * cos(phi * M_PI / 180.0);
  return xyz_pos(x,y,z);
}
polar_pos spherical_pos::convert_to_polar() {
  return polar_pos(rho,theta);
}

#endif // POSITION_H