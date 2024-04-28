// MotorController.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "PIDClass.h"

class MotorController {
  public:
    int pwmPin;
    int EncAPin;
    int EncBPin;
    int In1Pin;
    int In2Pin;
    int currentDir;
    volatile int pulses;
    int currentSpeed;
    bool currentDirection;
    PIDClass PID;

  public:
    MotorController(int pwmPin, 
                    int EncAPin, int EncBPin, 
                    int In1Pin, int In2Pin, 
                    float kp, float ki, float kd);
    void readEncoder();
    int getPulses();
    void resetPulses();
    void setSpeed(int speed);
    void setDirection(bool direction);
    int getDirection(float power);
    void stop();
    int getTargetSpeed();
    bool getCurrentDirection();
    void MotorController::controlMotor(float rpm, float target_rpm, float deltaTs);
};

#endif
