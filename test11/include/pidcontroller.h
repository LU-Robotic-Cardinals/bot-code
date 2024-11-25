#include "vex.h"
#include <cmath>
#include <string>
#include <vector>
#include "helpers.h"

class PIDController {
public:
    PIDController(double P, double I, double D,
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

        previous_error = error;

        return proportional + integral + derivative;
    }
    
    std::vector<double> values(){
      double dt = (Time.time() - last_time)/10;
      double prop = Kp * previous_error;
      double inte = Ki * previous_error * dt;
      double deri = Kd * (previous_error - previous_error) / dt;
      std::vector<double> array = {prop, inte, deri, previous_error};
      return array;
    }

private:
    double Kp, Ki, Kd;
    double Mp, Mi, Md;
    double integral, previous_error;
    double last_time;
    timer Time;
};