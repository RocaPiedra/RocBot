// MotorControlPID.ino

#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "src/MotorController.hpp"
#include "src/PIDClass.hpp"

// #define ENCA 2 // Sensor signal line A phase - YELLOW
// #define ENCB 3 // Sensor signal line B phase - GREEN
// #define PWM 5 // EN-A L298N PWM SIGNAL - BLUE
// #define IN1 6 // L298N detection direction 2 - PURPLE
// #define IN2 7 // L298N detection direction 1 - WHITE
#define MAXCPR 330 //1320 // PULSOS POR REVOLUCIÓN DEL MOTOR
#define WRAD 48 // RADIO EN mm
#define MAXRPM 330
#define REFRESHRATE 5 // ms

MotorController MotorFR("FR",5,2,4,6,7,0.8,0.1,1.0);
MotorController MotorFL("FL",10,3,9,11,12,0.8,0.1,1.0);

volatile int theta = 0; // specify pulsos as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
int target_value = 60;
float aux_timer = 0;
float change_value = 0;

unsigned long temp_ms;
int signal_period = 10000; // ms
float ChangeInputSignalTime=2;//signal_period/360;// para periodo de 10s // para señal cuadrada: 5; // seconds
float temporizadorCambioVelocidad=0;

int step=0;

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(MotorFR.EncAPin), ISRReadEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(MotorFL.EncAPin), ISRReadEncoderFL, RISING);
  Serial.println("SETUP FINISHED");  
}

void loop() {

  // set target pulsostion
  //int target = 1200;
  readSerialInput();
  int target = target_value;

  // time difference
  long currT = micros();
  float deltaTs = ((float) (currT - prevT))/( 1.0e6 );
  float deltaTms = ((float) (currT - prevT))/( 1.0e3 );
  float temporizadorCambioValor = fabs(((float) (currT - temporizadorCambioVelocidad))/( 1.0e6 )); // ms
  
  if (deltaTms >= REFRESHRATE){
      //Modifica las variables de la interrupción forma atómica
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){        
        MotorFR.updateRPM((float)MAXCPR, deltaTms);
        MotorFL.updateRPM((float)MAXCPR, deltaTms);
        prevT = currT;
      }    
    MotorFR.controlMotor(target, deltaTs);  
    MotorFL.controlMotor(target, deltaTs);    
  }
  squaredMultiInputSignal(temporizadorCambioValor, currT);
}

void ISRReadEncoderFR(){
  MotorFR.readEncoder();
}

void ISRReadEncoderFL(){
  MotorFL.readEncoder();
}

void readSerialInput(){
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    float userInput = Serial.read();               // read user input
      
      if(userInput == 'g'){                  // if we get expected value 
   // read the input pin
            Serial.println(MotorFL.GetMotorState() + MotorFR.GetMotorState());
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
        if (target_value <= 60){
          target_value = 180;
        }else{
          target_value = 60;
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