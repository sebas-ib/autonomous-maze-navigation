#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {
  /*Initialize values by copying and pasting from PD controller, then declaring for
  the three new variables.*/
  _kp = kp;
  _ki = ki;
  _kd = kd;

  _minOutput = minOutput;
  _maxOutput = maxOutput;
  _clamp_i = clamp_i;

  _error = 0;
  _previous_error = 0;
  _accumulated_error = 0;

  _proportionalOut = 0;
  _integralOut = 0;
  _derivativeOut = 0;
  _clampOut = 0;

  _prev_time = millis();
}

double PIDcontroller::update(double value, double target_value){
  /*Now copy and paste your PD controller. To implement I component,
  keep track of accumulated error, use your accumulated error in the constrain
  function for the integral, multiply ki by your integral, then add your p, d,
  and i components.
  
  Note: Do not just put all of the integral code at the end of PD component. Think
  about step by step how you can integrate these parts into your PDController
  code.*/

  // compute the error
  _error = target_value - value;

  // compute dt
  _curr_time = millis();
  float dt = (_curr_time - _prev_time);


  _prev_time = _curr_time;

  // the proportional Term
  _proportionalOut = _kp * _error;

  // integral
  _accumulated_error += _error * dt;  // sum of error over time
  _accumulated_error = constrain(_accumulated_error, -_clamp_i, _clamp_i); 
  _integralOut = _ki * _accumulated_error; 


  // derivative term
  float de = _error - _previous_error;
  _derivativeOut = _kd * (de / dt);

  

  // combine terms and constrain
  _clampOut = constrain(_proportionalOut + _integralOut + _derivativeOut, _minOutput, _maxOutput);

  // update previous error for next cycle
  _previous_error = _error;

  return _clampOut;

}
