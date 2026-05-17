// PIDClass.cpp
#include "Arduino.h"
#include "PIDClass.hpp"
/*
@brief PID Class constructor
@param _kp proportional constant for the PID
@param _ki integral constant for the PID
@param _kd derivative constant for the PID
*/
PIDClass::PIDClass(float _kp, float _ki, float _kd):
kp(_kp), ki(_ki), kd(_kd){ 
}

float PIDClass::CalculateOutput(float rpm, float target_rpm, float deltaTs){
    // error: positive when we need to go faster
    e = target_rpm - rpm;
    
    // derivative
    dedt = (e - e_prev) / deltaTs;
    
    // integral
    eint = eint + e * deltaTs;
    
    // store previous error
    e_prev = e;
    
    // control signal (raw)
    float output = kp * e + kd * dedt + ki * eint;
    
    return output;
}

float PIDClass::ScaleOutput(float output){
    // Scale the PID output to PWM range
    // If output is 30 (error for Kp=1), we want PWM around 100-150
    // So scale = 5 means 30 * 5 = 150 PWM
    return output * output_scale;
}

float PIDClass::FilterOutput(float output){    
    // Apply output scaling first
    float scaled = ScaleOutput(output);
    
    // motor power
    pwr = fabs(scaled);
    if(pwr > 255){
        pwr = 255;
    }

    // IIR low-pass filter
    pwr_filt = filter_1 * pwr_filt + filter_2 * pwr + filter_3 * pwr_prev;

    pwr_prev = pwr;
    
    return pwr_filt;
}

float PIDClass::GetPower(){
    return pwr;
}

float PIDClass::GetPowerFiltered(){
    return pwr_filt;
}