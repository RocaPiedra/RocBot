// MotorController.cpp
#include "Arduino.h"
#include "MotorController.hpp"
#include "PIDClass.hpp"

MotorController::MotorController(String motorPos, int PWMPin, 
                                int EncAPin, int EncBPin, 
                                int In1Pin, int In2Pin, 
                                float kp, float ki, float kd)
    :motorPos(motorPos), PWMPin(PWMPin),
    EncAPin(EncAPin), EncBPin(EncBPin),
    In1Pin(In1Pin), In2Pin(In2Pin), PID(kp,ki,kd) {
    pinMode(PWMPin, OUTPUT);
    pinMode(EncAPin, INPUT);
    pinMode(EncBPin, INPUT);
    pinMode(In1Pin, OUTPUT);
    pinMode(In2Pin, OUTPUT);
}

void MotorController::readEncoder(){
  int b = digitalRead(EncBPin);
  if(b > 0){
    this->pulses++;
  }
  else{
    this->pulses--;
  }
}

int MotorController::GetPulses(){
  return this->pulses;
}

void MotorController::resetPulses(){
  pulses = 0;
}

int MotorController::GetDirection(float power){
  if(power<0){
      this->currentDir = -1; // Backward
      return -1;
    }else{  
      this->currentDir = 1; // Forward
      return 1;
    }
}

void MotorController::SetSpeed(int speed) {
  analogWrite(PWMPin, speed);
  currentSpeed = speed;
}

void MotorController::SetDirection(bool direction) {
  digitalWrite(this->In1Pin, direction ? HIGH : LOW);
  digitalWrite(this->In2Pin, direction ? LOW : HIGH);
  currentDirection = direction;
}

void MotorController::stop() {
  analogWrite(PWMPin, 0);
}

int MotorController::GetTargetSpeed() {
  return currentSpeed;
}

bool MotorController::GetCurrentDirection() {
  return currentDirection;
}

void MotorController::controlMotor(float target_rpm, float deltaTs){
  this->target_rpm = target_rpm;
  float output = this->PID.CalculateOutput(this->current_rpm, target_rpm, deltaTs);
  // motor direction
  this->SetDirection(output>0);
  // signal the motor with the filtered output
  this->SetSpeed(this->PID.FilterOutput(output));

}

void MotorController::updateRPM(float max_cpr, float deltaTms){
  
  this->current_rpm = ((float)MotorController::GetPulses()/(float)max_cpr*1000/deltaTms)*60;
  this->filtered_rpm = 0.854*filtered_rpm + 0.0728*current_rpm + 0.0728*previous_rpm; //review filter
  this->previous_rpm = current_rpm;
  this->resetPulses();

}

String MotorController::GetMotorState(){
  
  return "&" + String(this->motorPos) + 
          ";target:" + String(this->target_rpm)+
          ";rpm:" + String(this->current_rpm)+
          ";rpm_filt:" + String(this->filtered_rpm)+
          ";pwr:" + String(this->currentDir * this->PID.GetPower())+
          ";pwr_filt:" + String(this->currentDir * this->PID.GetPowerFiltered());

}