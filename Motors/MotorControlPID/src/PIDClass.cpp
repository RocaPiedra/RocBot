// PIDClass.cpp
#include "Arduino.h"
#include "PIDClass.h"

PIDClass::PIDClass(float _kp, float _ki, float _kd){
    kp = _kp;
    ki = _ki;
    kd = _kd;    
}

float PIDClass::CalculateOutput(float rpm, float target_rpm, float deltaTs){
    // error
    e = rpm - target_rpm; // original calculation
    // derivative
    dedt = (e-e_prev)/(deltaTs);
    // integral
    eint = eint + e*deltaTs;
    // store previous error
    e_prev = e;
    // control signal
    return kp*e + kd*dedt + ki*eint;
}

float PIDClass::FilterOutput(float output){    
    // motor power
    pwr = fabs(output);
    if( pwr > 255 ){
      pwr = 255;
    }

    pwr_filt = filter_1*pwr_filt + filter_2*pwr + filter_3*pwr_prev;

    pwr_prev = pwr;
    
    return pwr_filt;
}

float PIDClass::GetPower(){
    return pwr;
}

float PIDClass::GetPowerFiltered(){
    return pwr_filt;
}