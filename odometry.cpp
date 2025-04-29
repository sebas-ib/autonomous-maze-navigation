#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include "odometry.h"
#include "printOLED.h"
using namespace Pololu3piPlus32U4;

#define PI 3.14159

PrintOLED printOLED1;

Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning){
  _diaL = diaL;
  _diaR = diaR;
  _w = w;
  _nL = nL;
  _nR = nR;
  _gearRatio = gearRatio;
  _deadreckoning = dead_reckoning;

  _x = 0;
  _y = 0;
  _theta = 0;

  _left_encoder_counts_prev = 0;
  _right_encoder_counts_prev = 0;

  if(_deadreckoning){ // if using dead reckoning, initialize and calibrate IMU
    Wire.begin();
    _imu.init();
    _imu.enableDefault();

    //calibrate IMU
    int total = 0;
    for (int i = 0; i < 100; i++)
    {
      _imu.readGyro();
      total += _imu.g.z;
      delay(1);
    }
    _IMUavg_error = total / 100;  
  }
}

// USE ODOMETRY FORMULAS TO CALCULATE ROBOT'S NEW POSITION AND ORIENTATION
void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta){

  int change_in_left = left_encoder_counts - _left_encoder_counts_prev;
  int change_in_right = right_encoder_counts - _right_encoder_counts_prev;

  float distance_per_count_L = (PI * _diaL) / (_nL * _gearRatio);
  float distance_per_count_R = (PI * _diaR) / (_nR * _gearRatio);

  float dL = change_in_left * distance_per_count_L;
  float dR = change_in_right * distance_per_count_R;

  float delta_d = (dL + dR) / 2.0;

  if (_deadreckoning) {
    // IF USING dead reckoning, GET THE ANGLE _theta FROM IMU
    _imu.readGyro();
    float gyro_z = _imu.g.z - _IMUavg_error;
    _theta += gyro_z;
  } else {
    // OTHERWISE, CALCULATE THE ANGLE _theta FROM ENCODERS DATA BASED ON THE FORMULA
    float dTheta = (dR - dL) / _w;
    _theta += dTheta;
  }

  // CALCULATE _x BASED ON THE FORMULA FROM THE LECTURES
  _x += delta_d * cos(_theta);

  // CALCULATE _y BASED ON THE FORMULA FROM THE LECTURES
  _y += delta_d * sin(_theta);

  // Update cumulative x, y, and theta
  x = _x;
  y = _y;
  theta = _theta;

  // Print values to OLED
  printOLED1.print_odom(x, y, theta);

  // Print values to Serial Monitor
  // Serial.print(" x:  "); 
  // Serial.println(x); 
  // Serial.print(" y:  ");
  // Serial.println(y);
  // Serial.print(" theta:  ");
  // Serial.println(theta);

  // Save the current encoder values for next update
  _left_encoder_counts_prev = left_encoder_counts;
  _right_encoder_counts_prev = right_encoder_counts;
}
