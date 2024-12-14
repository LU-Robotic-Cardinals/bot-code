#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
// #include "helpers.h"
// #include "classes.h"

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(double P = 1, double I = 0, double D = 0,
    double max_p = 0, double max_I = 0, double max_D = 0) : 
    Kp(P), Ki(I), Kd(D),
    Mp(max_p), Mi(max_I), Md(max_D),
    integral(0.0), previous_error(0.0) {
      last_time = Time.time();
    }

    double calculate(double error) {
        // The divide by 10 is to keep things under controll
        double dt = (Time.time() - last_time)/10; 
        last_time = Time.time();

        double proportional = Kp * error;
        if (Mp != 0 && fabs(proportional) > fabs(Mp)) {
          proportional = fabs(Mp) * getSign(proportional);
        }

        integral += Ki * error * dt;
        if (Mi != 0 && fabs(integral) > fabs(Mi)) {
          integral = fabs(Mi) * getSign(integral);
        }

        double derivative = Kd * (error - previous_error) / dt;
        if (Md != 0 && fabs(derivative) > fabs(Md)) {
          derivative = fabs(Md) * getSign(derivative);
        }


        last_prop = proportional;
        last_inte = integral;
        last_deri = derivative;
        previous_error = error;
        last_dt = dt;

        return proportional + integral + derivative;
    }
    
    std::vector<double> values(){
      std::vector<double> array = {
        previous_error, last_prop, last_inte, last_deri};
        // std::cout << last_deri << "\n\n";
      return array;
    }

private:
    double Kp, Ki, Kd;
    double Mp, Mi, Md;
    double integral, previous_error;
    double last_time;
    timer Time;

    double last_dt = 1;
    double last_prop = 0;
    double last_inte = 0;
    double last_deri = 0;

};

#endif // PIDCONTROLLER