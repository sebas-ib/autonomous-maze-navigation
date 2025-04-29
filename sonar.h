#ifndef Sonar_h
#define Sonar_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class Sonar{
  public:
    Sonar(int pin);
    float readDist();
    
  private:
    int _pin;
};

#endif
