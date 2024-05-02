#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDClass {
  public:
    float kp; 
    float ki; 
    float kd;
    float filter_1 = 0.854;
    float filter_2 = 0.0728; 
    float filter_3 = 0.0728;
    float e = 0;
    float eint = 0;
    float dedt = 0;
    float e_prev = 0;
    float pwr = 0; 
    float pwr_filt = 0; 
    float pwr_prev = 0;

  public:
    PIDClass(float _kp, float _ki, float _kd);
    float CalculateOutput(float rpm, float target_rpm, float deltaTs);
    float FilterOutput(float output);
    float GetPower();
    float GetPowerFiltered();
};

#endif
