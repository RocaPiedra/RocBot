// MotorController.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "PIDClass.h"

class MotorController {
  public:
    String motorPos;
    int pwmPin;
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
    MotorController(String motorPos, int pwmPin, 
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
    void controlMotor(float target_rpm, float deltaTs);
    void updateRPM(float max_cpr, float deltaTms);
    String getMotorState();
};

#endif
