// PIDClass.hpp
#ifndef PIDCLASS_H
#define PIDCLASS_H

class PIDClass {
  public:
    float kp, ki, kd;
    
    // Filter coefficients for IIR low-pass
    float filter_1 = 0.854;
    float filter_2 = 0.0728;
    float filter_3 = 0.0728;
    
    // PID variables
    float e = 0;        // current error
    float e_prev = 0;   // previous error
    float dedt = 0;     // derivative of error
    float eint = 0;     // integral of error
    
    // Output
    float pwr = 0;      // raw power (before filter)
    float pwr_filt = 0; // filtered power
    float pwr_prev = 0;
    
    // Output scaling factor
    // PID output needs to be scaled to 0-255 PWM range
    float output_scale = 10.0;  // Default: multiply output by this
    
    PIDClass(float _kp, float _ki, float _kd);
    
    // Calculate PID output with proper scaling
    float CalculateOutput(float rpm, float target_rpm, float deltaTs);
    
    // Apply output scaling
    float ScaleOutput(float output);
    
    // Filter the output
    float FilterOutput(float output);
    
    // Get power values
    float GetPower();
    float GetPowerFiltered();
    
    // Reset integral
    void resetIntegral() { eint = 0; }
};

#endif