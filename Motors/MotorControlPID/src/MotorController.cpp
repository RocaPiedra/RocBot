// MotorController.cpp
#include "Arduino.h"
#include "MotorController.h"
#include "PIDClass.h"

MotorController::MotorController(String motorPos, int pwmPin, 
                                int EncAPin, int EncBPin, 
                                int In1Pin, int In2Pin, 
                                float kp, float ki, float kd)
    :motorPos(motorPos), pwmPin(pwmPin),
    EncAPin(EncAPin), EncBPin(EncBPin),
    In1Pin(In1Pin), In2Pin(In2Pin), PID(kp, kd, ki) {
    pinMode(pwmPin, OUTPUT);  
    pinMode(EncAPin, INPUT);
    pinMode(EncBPin, INPUT);
    pinMode(In1Pin, OUTPUT);
    pinMode(In2Pin, OUTPUT);
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

void MotorController::controlMotor(float target_rpm, float deltaTs){
  this->target_rpm = target_rpm;
  float output = this->PID.CalculateOutput(this->current_rpm, target_rpm, deltaTs);
  // motor direction
  this->setDirection(output>0);
  // signal the motor with the filtered output
  this->setSpeed(this->PID.FilterOutput(output));

}

void MotorController::updateRPM(float max_cpr, float deltaTms){
  
  this->current_rpm = ((float)MotorController::getPulses()/(float)max_cpr*1000/deltaTms)*60;
  this->filtered_rpm = 0.854*filtered_rpm + 0.0728*current_rpm + 0.0728*previous_rpm; //review filter
  this->previous_rpm = current_rpm;
  MotorController::resetPulses();

}

String MotorController::getMotorState(){
  
  return "&" + String(this->motorPos) + 
          ";target:" + String(this->target_rpm)+
          ";rpm:" + String(this->current_rpm)+
          ";rpm_filt:" + String(this->filtered_rpm)+
          ";pwr:" + String(this->PID.GetPower())+
          ";pwr_filt:" + String(this->PID.GetPowerFiltered());

}