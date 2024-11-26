#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

struct OdomWheel {
private:
    vex::rotation Encoder;
    double last_encoder = Encoder.position(degrees);
    double wheel_radius; // Wheel radius
    double wheel_angle; // Angle from arbitrary "front"
    // Right handed world with z-axis normal to floor
    double wheel_x_offset; // When facing "front", positive x is to the right, angle 0
    double wheel_y_offset; // When facing "front", positive y is forward, angle 90

    vex::inertial Inertial;
    double last_rot = Inertial.rotation(); // Last rotation value in degrees
    
    timer Time; // Time object
    double last_time = Time.time(); // Last time value

    double x_pos = 0; // Current position in x-axis
    double y_pos = 0; // Current position in y-axis

public:

    OdomWheel(vex::rotation encoder_obj, vex::inertial inertial_obj, double wheel_radius_size = 1, 
    double wheel_center_angle = 0, double wheel_offset_x = 0, double wheel_offset_y = 0): 
    
    Encoder(encoder_obj), Inertial(inertial_obj), wheel_radius(wheel_radius_size),
    wheel_angle(wheel_center_angle), wheel_x_offset(wheel_offset_x), wheel_y_offset(wheel_offset_y) {}

    
    double update() {
        // It is assumed that this update function is called sufficiently
        // often so that the math works correctly. IMO, at least 30 Hz
        // depending on anticiated bot speed


        double current_time = Time.time();
        double dt = current_time - last_time;

        double current_rot = Inertial.rotation();
        // Delta rotations of bot since last update
        double dr = current_rot - last_rot;
        double avg_rot = (current_rot + last_rot)/2;

        double current_encoder = Encoder.position(degrees);
        double de = current_encoder - last_encoder;

        // circumference of the encoder wheel, in inches
        double wheel_circumference = 2.0 * M_PI * wheel_radius;

        // Radius of encoder from rot center, in inches
        double radius = pow(pow(wheel_x_offset,2)+pow(wheel_y_offset,2),0.5);

        // Length of the path that the encoder traced out, in inches
        double path_circumference = radius * (dr / 360) * 2 * M_PI;

        // Percentage of path_circumference that the encoder will incur, -1 to 1
        double angle_coef = cos(  (wheel_angle  +  90.0  -  (atan2(wheel_y_offset,wheel_x_offset)/M_PI*180.0))   / 180.0*M_PI);

        // The number of incorrect rotations the encoder incurred since last update
        // due to the bot spinning
        double wrong_rots_from_spinning = - (path_circumference * angle_coef) / wheel_circumference;
        
        // Change in distance detected by encoder
        double dd = ((de/360.0) - wrong_rots_from_spinning) * wheel_circumference;

        // Vector decomposition along cartesian axies
        double dx = cos(avg_rot + wheel_angle) * dd;
        double dy = sin(avg_rot + wheel_angle) * dd;
        
        // Calculate new dist
        x_pos += dx;
        y_pos += dy;

        // If purely spinning and not moving in any direction,
        // the following code will try to predict the correct distance from
        // center of rotation. This should be made into a standalone function
        // for use elsewhere in the code

        // Not finished, need to account for the wheel being at an angle relative to bot
        // with angle_coef
        // double calculated_radius = (de / (360.0 * angle_coef) * wheel_circumference) / ( (dr/360.0) * 2 * M_PI);

        std::cout << "x_pos: " << x_pos << "\ny_pos: " << y_pos << "\n";
        // std::cout << "dr: " << dr/360.0 << "\n";
        // std::cout << "de: " << de/360.0 << "\n";
        // std::cout << "Angle: " << current_rot << "\n";
        // std::cout << "Wrong: " << wrong_rots_from_spinning << "\n";
        // std::cout << "Error: " << (de/360.0) + wrong_rots_from_spinning << "\n";
        // std::cout << "Atan:" << (atan2(wheel_y_offset,wheel_x_offset)/M_PI*180.0) << "\n";
        // std::cout << wheel_angle << "\n";
        // std::cout << calculated_radius << "\n";
        // std::cout << angle_coef << "\n";



        last_encoder = current_encoder;
        last_time = current_time;
        last_rot = current_rot;
        return 0;
    }

};