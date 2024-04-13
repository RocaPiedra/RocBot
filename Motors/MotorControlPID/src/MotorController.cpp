// MotorController.cpp
#include "Arduino.h"
#include "MotorController.h"

MotorController::MotorController(int pwmPin, int EncAPin, int EncBPin, int In1Pin, int In2Pin) {
  this->pwmPin = pwmPin;
  this->EncAPin = EncAPin;
  this->EncBPin = EncBPin;
  this->In1Pin = In1Pin;
  this->In2Pin = In2Pin;
  pinMode(pwmPin, OUTPUT);  
  pinMode(EncAPin,INPUT);
  pinMode(EncBPin,INPUT);
  pinMode(In1Pin,OUTPUT);
  pinMode(In2Pin,OUTPUT);
  // attachInterrupt(digitalPinToInterrupt(EncAPin), MotorController::readEncoder, RISING); // Can't do this here
  
}

void MotorController::readEncoder(){
  int b = digitalRead(EncBPin);
  if(b > 0){
    MotorController::pulses++;
  }
  else{
    MotorController::pulses--;
  }
}

int MotorController::getPulses(){
  return MotorController::pulses;
}

void MotorController::resetPulses(){
  pulses = 0;
}

int MotorController::getDirection(float power){
  if(power<0){
      this->currentDir = -1; // Backward
      return -1;
    }else{  
      this->currentDir = 1; // Forward
      return 1;
    }
}

void MotorController::setSpeed(int speed) {
  analogWrite(pwmPin, speed);
  currentSpeed = speed;
}

void MotorController::setDirection(bool direction) {
  digitalWrite(this->In1Pin, direction ? HIGH : LOW);
  digitalWrite(this->In2Pin, direction ? LOW : HIGH);
  currentDirection = direction;
}

void MotorController::stop() {
  analogWrite(pwmPin, 0);
}

int MotorController::getTargetSpeed() {
  return currentSpeed;
}

bool MotorController::getCurrentDirection() {
  return currentDirection;
}