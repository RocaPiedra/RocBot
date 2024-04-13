// MotorController.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

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

  public:
    MotorController(int pwmPin, int EncAPin, int EncBPin, int In1Pin, int In2Pin);
    void readEncoder();
    int getPulses();
    void resetPulses();
    void setSpeed(int speed);
    void setDirection(bool direction);
    int getDirection(float power);
    void stop();
    int getTargetSpeed();
    bool getCurrentDirection();
};

#endif
