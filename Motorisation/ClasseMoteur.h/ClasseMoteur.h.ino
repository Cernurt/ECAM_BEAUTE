#ifndef ClasseMoteur_h
#define ClasseMoteur_h
#include <AFMotor.h>
#include <TimerOne.h>
#include "Arduino.h" 
class ClasseMoteur{
public:
  ClasseMoteur(int nmr, String type, int pinEncoder);
  
  void myFunction(int blinkRate);

  double speedM;
  double cibleVitesse;
  volatile int hallTicks;
  double tempTicks;
  double ttlTicks;
  
private:
  float _kp;
  float _ki;
  float _kd;
 
  
};
#endif
