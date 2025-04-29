#ifndef Odometry_h
#define Odometry_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class Odometry{
  public:
    Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning);
    void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta);
    
  private:
    float _diaL;
    float _diaR;
    float _w;
    int _nL;
    int _nR;
    int _gearRatio;
    IMU _imu;
    float _IMUavg_error;
    bool _deadreckoning;
    int _left_encoder_counts_prev;
    int _right_encoder_counts_prev;

    double _x;
    double _y;
    double _theta;
};

#endif
