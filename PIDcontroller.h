#ifndef PIDcontroller_h
#define PIDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller{
  public:
    PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i);
    double update(double value, double target_value);
    
  private:
    /*Add variables from your PDController, then add varaibles for ki, your clamp
    for i component, and accumulated error.*/
    // gains
    float _kp;
    float _ki;
    float _kd;

    // output limits
    double _minOutput;
    double _maxOutput;

    // integral clamp (i believe)
    double _clamp_i;

    // internal state variables
    float _error;
    float _previous_error;
    float _accumulated_error;   // this is for the integral 

    float _proportionalOut;
    float _integralOut;
    float _derivativeOut;
    double _clampOut;

    // the timing variables for derivative calculation
    unsigned long _prev_time;
    unsigned long _curr_time;

};

#endif
