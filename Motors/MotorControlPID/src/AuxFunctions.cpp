#include "AuxFunctions.hpp"


float RPMtoMpS(float rpm, float wheel_radius){
  return (2 * PI * (wheel_radius/1000)) / 60 * rpm;
}

void plotter_logger(int target, float real, float power, int error){
  Serial.println("");
  Serial.print("targetRPM:");
  Serial.print(target);
  Serial.print("|xy ");
  Serial.print("realRPM:");
  Serial.print(real);
  Serial.print("|xy ");
  Serial.print("PWR-output:");
  Serial.print(power);
  Serial.print("|xy ");
  // Serial.print("error:");
  // Serial.print(error);
  // Serial.print("|xy ");
  // Serial.print(" - E:");
  // Serial.print(e);
  // Serial.print(" - dEdt:");
  // Serial.print(dedt);
  // Serial.print(" - E Integral:");
  // Serial.print(eintegral);
  // Serial.print(" - PrevT:");
  // Serial.print(prevT);
  // Serial.print(" - CurrT:");
  // Serial.print(micros());
  // Serial.println("|xy ");

}