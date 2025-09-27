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
  // error
  _error = target_value - value;

  // dt in seconds (guard against zero)
  _curr_time = millis();
  unsigned long elapsed_ms = _curr_time - _prev_time;
  double dt = elapsed_ms / 1000.0;    // convert ms → s
  _prev_time = _curr_time;

  // P term
  _proportionalOut = _kp * _error;

  // I term (accumulate only when dt > 0)
  if (dt > 0.0) {
    _accumulated_error += _error * dt;
    _accumulated_error = constrain(_accumulated_error, -_clamp_i, _clamp_i);
  }
  _integralOut = _ki * _accumulated_error;

  // D term (safe if dt == 0)
  double de = _error - _previous_error;
  _derivativeOut = (dt > 0.0) ? _kd * (de / dt) : 0.0;

  // Sum & clamp
  _clampOut = constrain(_proportionalOut + _integralOut + _derivativeOut, _minOutput, _maxOutput);

  // keep for next cycle
  _previous_error = _error;

  return _clampOut;
}
