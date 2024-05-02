// MotorController.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "PIDClass.hpp"

class MotorController {
  public:
    String motorPos;
    int PWMPin;
    int EncAPin;
    int EncBPin;
    int In1Pin;
    int In2Pin;
    int currentDir;
    volatile int pulses;
    int currentSpeed;
    bool currentDirection;
    float current_rpm=0;
    float filtered_rpm=0;
    float previous_rpm=0;
    float target_rpm=0;
    PIDClass PID;

  public:
    /*
    @brief Motor controller class constructor
    @param motorPos two character string defining the position of the motor FL, FR, BL, BR
    @param EncAPin pin for the first encoder signal
    @param EncBPin pin for the second encoder signal
    @param In1Pin motor driver direction pin 1
    @param In2Pin motor driver direction pin 2
    @param kp proportional constant for the PID
    @param ki integral constant for the PID
    @param kd derivative constant for the PID
    */  
    MotorController(String motorPos, int pwmPin, 
                    int EncAPin, int EncBPin, 
                    int In1Pin, int In2Pin, 
                    float kp, float ki, float kd);
    void readEncoder();
    int GetPulses();
    void resetPulses();
    void SetSpeed(int speed);
    void SetDirection(bool direction);
    int GetDirection(float power);
    void stop();
    int GetTargetSpeed();
    bool GetCurrentDirection();
    void controlMotor(float tarGet_rpm, float deltaTs);
    void updateRPM(float max_cpr, float deltaTms);
    String GetMotorState();
};

#endif
