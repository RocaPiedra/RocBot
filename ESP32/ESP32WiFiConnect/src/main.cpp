#include <Arduino.h>
#include "MotorController.hpp"
#include "PIDClass.hpp"
// #include <Wifi.h>
// #include <WifiMulti.h>
// #include "ssid.hpp"
// #include <util/atomic.h> // For the ATOMIC_BLOCK macro


// WiFiMulti myWiFi;

#define MAXCPR 330 //1320 // PULSOS POR REVOLUCIÓN DEL MOTOR
#define WRAD 48 // RADIO EN mm
#define MAXRPM 330
#define REFRESHRATE 5 // ms
#define NUMMOTORS 2

MotorController MotorFR("FR",14,12,13,27,26,0.8,1.0,0.1);
MotorController MotorFL("FL",32,35,34,33,25,0.8,1.0,0.1);
MotorController motors[NUMMOTORS] = {MotorFL,MotorFR};

long prevT[NUMMOTORS] = {0,0};
// int target_value[NUMMOTORS] = {60,60};
int target_value = 60;

unsigned long temp_ms;
int signal_period = 10000; // ms
float ChangeInputSignalTime=2;//signal_period/360;// para periodo de 10s // para señal cuadrada: 5; // seconds
float temporizadorCambioVelocidad=0;

int step=0;

void setup() {
  // TCCR1B = TCCR1B & B11111000 | B00000101;
  // EIFR |= (1 << INTF1);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(MotorFL.EncAPin), ISRReadEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(MotorFR.EncAPin), ISRReadEncoderFR, RISING);
  Serial.println("SETUP FINISHED");  
  // // pinMode(LED_BUILTIN, OUTPUT); // STATUS INDICATOR
  // myWiFi.addAP(WIFI_SSID_1, WIFI_PASSWORD);
  // myWiFi.addAP(WIFI_SSID_2, WIFI_PASSWORD);
  // while (myWiFi.run() != WL_CONNECTED){
  //   delay(100);
  // }
  // Serial.println("connected to " + String(WiFi.SSID()));
}

void loop() {

  // digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
  // Serial.println("alive: connected to " + String(WiFi.SSID()));
    // delay(1000);
  readSerialInput();
  int target = target_value;
  long currT = 0;
  float temporizadorCambioValor = 0;
  
  for(int id = 0; id < NUMMOTORS; id++){
    // measure elapsed time
    currT = micros();
    float deltaTms = ((float) (currT - prevT[id]))/( 1.0e3 );
    temporizadorCambioValor = fabs(((float) (currT - temporizadorCambioVelocidad))/( 1.0e6 )); // ms

    if (deltaTms >= REFRESHRATE){
      // ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        motors[id].updateRPM((float)MAXCPR, deltaTms);
      // }
      prevT[id] = currT;     
      motors[id].controlMotor(target, deltaTms*1000);
    }
  }
  squaredInputSignal(temporizadorCambioValor, currT);
}

void ISRReadEncoderFR(){
  MotorFR.readEncoder();
  Serial.println("Flanco de subida en FR: Pin " + String(MotorFR.EncAPin) + " - Pulsos: " + String(MotorFR.GetPulses()));
}

void ISRReadEncoderFL(){
  MotorFL.readEncoder();
  Serial.println("Flanco de subida en FL: Pin " + String(MotorFL.EncAPin) + " - Pulsos: " + String(MotorFL.GetPulses()));
}

void readSerialInput(){
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    float userInput = Serial.read();               // read user input
      
      if(userInput == 'g'){                  // if we get expected value 
   // read the input pin
        for(int id = 0; id < NUMMOTORS; id++){
          Serial.println(motors[id].GetMotorState() + MotorFR.GetMotorState());
        }
      } else {
      int rpm_val = Serial.parseInt();
      if (rpm_val != 0) {
        if (fabs(rpm_val) > MAXRPM){
          if (rpm_val > 0){
          rpm_val = MAXRPM;
          }
          else{
            rpm_val = -MAXRPM;
          }
        }
      target_value = rpm_val;
      Serial.print("Changing target value to: ");
      Serial.println(target_value);
      }
    }
  }
}

float squaredInputSignal(float elapsed_time, float currT){
    if (fabs(elapsed_time) >= ChangeInputSignalTime){
        temporizadorCambioVelocidad = currT;
        if (target_value <= 0){
          target_value = 120;
        }else{
          target_value = 0;
        }
    }
}

float squaredMultiInputSignal(float elapsed_time, float currT){
    if (fabs(elapsed_time) >= ChangeInputSignalTime){
        temporizadorCambioVelocidad = currT;
        if (target_value == 60){
          target_value = 180;
        }else if (target_value == 180){
          target_value = 240;
        }else if (target_value == 240){
          target_value = -60;
        }else if (target_value == -60){
          target_value = -180;
        }else if (target_value == -180){
          target_value = -240;
        }else if (target_value == -240){
          target_value = 0;
        } else {
          target_value = 60;
        }
    }
}

float sinusoidalInputSignal(float elapsed_time, float currT, int step){
    if (fabs(elapsed_time) >= ChangeInputSignalTime){
      temporizadorCambioVelocidad = currT;
      float t = step * signal_period;
      // target_value = (600*sin(PI/180)*t/(signal_period+360));
      // Serial.println("t: " + String(t) + 
      // " elapsed_time: " + String(elapsed_time) + 
      // " target_value: " + String(target_value));
    }
}
