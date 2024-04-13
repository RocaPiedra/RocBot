// MotorControlPID.ino

#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "src/MotorController.h"

// #define ENCA 2 // Sensor signal line A phase - YELLOW
// #define ENCB 3 // Sensor signal line B phase - GREEN
// #define PWM 5 // EN-A L298N PWM SIGNAL - BLUE
// #define IN1 6 // L298N detection direction 2 - PURPLE
// #define IN2 7 // L298N detection direction 1 - WHITE
#define MAXCPR 330 //1320 // PULSOS POR REVOLUCIÓN DEL MOTOR
#define WRAD 48 // RADIO EN mm
// #define PI 3.1416
#define MAXRPM 330
#define REFRESHRATE 50 // ms

MotorController MotorFL(5,2,3,6,7);

volatile int theta = 0; // specify pulsos as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float e = 0;
float dedt = 0;
float eintegral = 0;
int target_value = 60;
float aux_timer = 0;
float change_value = 0;

int vel_lineal = 0;
int vel_angular = 0;

float pwr=0;
float pwr_filt=0;
float pwr_prev=0;
float rpm=0;
float rpm_filt=0;
float rpm_prev=0;

unsigned long temp_ms;
int signal_period = 10000; // ms
float ChangeInputSignalTime=2;//signal_period/360;// para periodo de 10s // para señal cuadrada: 5; // seconds
float temporizadorCambioVelocidad=0;

int step=0;

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(MotorFL.EncAPin), ISRReadEncoderFL, RISING);
  Serial.println("SETUP FINISHED");
  
}

void loop() {

  // set target pulsostion
  //int target = 1200;
  readSerialInput();
  int target = target_value;
  // float rpm;
  float rps;
  float revoluciones;

  // PID constants
  float kp = 0.8; //0.05
  float kd = 0.1; // 0.025;
  float ki = 1.0;// 1.0;

  // time difference
  long currT = micros();
  float deltaTs = ((float) (currT - prevT))/( 1.0e6 );
  float deltaTms = ((float) (currT - prevT))/( 1.0e3 );
  float temporizadorCambioValor = fabs(((float) (currT - temporizadorCambioVelocidad))/( 1.0e6 )); // ms
  // float pwr;
  
  if (deltaTms >= REFRESHRATE){
      //Modifica las variables de la interrupción forma atómica
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        revoluciones = (float)MotorFL.getPulses()/(float)MAXCPR;
        rps = revoluciones*1000/deltaTms;
        rpm = rps*60;
        rpm_filt = 0.854*rpm_filt + 0.0728*rpm + 0.0728*rpm_prev;
        rpm_prev = rpm;
        Serial.println(" - RPM: " + String(rpm) + " - pulsos: " + String(MotorFL.getPulses()) + " - revoluciones: " + String(revoluciones) + " - RPS: " + String(rps) + " - deltaTs: " + String(deltaTs) + " - deltaTms: " + String(deltaTms));
        prevT = currT;
        MotorFL.resetPulses();
      }
    // error
    e = rpm - target; // original calculation
    // e = rpm_filt - target; // filtered calculation -> makes no sense, error must be based on the real value

    // derivative
    dedt = (e-eprev)/(deltaTs);

    // integral
    eintegral = eintegral + e*deltaTs;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    pwr = fabs(u);
    if( pwr > 255 ){
      pwr = 255;
    }
    pwr_filt = 0.854*pwr_filt + 0.0728*pwr + 0.0728*pwr_prev;
    pwr_prev = pwr;
    
    // motor direction
    MotorFL.setDirection(u>0);
    // signal the motor
    MotorFL.setSpeed(pwr_filt);
    // store previous error
    eprev = e;

    Serial.println(" - Speed: " + String(pwr_filt) + " - Target: " + String(target));
        
    }    
    // if (deltaTms >= REFRESHRATE){
    //   Serial.println("Current Step: " + String(step) + 
    // " - target: " + String(target_value));
        // logger(target, rpm, pwr, e);  
    // }

    // sinusoidalInputSignal(temporizadorCambioValor, currT, step);
    // if (step >= 360){
    //   step = 0;
    // } else {
    //   step += 1;
    // }
    // Serial.println("Current Step: " + String(step) + 
    // " - target: " + String(target_value));
    squaredMultiInputSignal(temporizadorCambioValor, currT);

}

void ISRReadEncoderFL(){
  MotorFL.readEncoder();
}
  // motor direction
int getMotorDirection(float u){
    if(u<0){
      return -1;
    }else{      
      return 1;
    }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readSerialInput(){
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    float userInput = Serial.read();               // read user input
      
      if(userInput == 'g'){                  // if we get expected value 
   // read the input pin
            Serial.println("target:"+ String(target_value) +
            ";rpm:"+ String(rpm)+
            ";rpm_filt:"+ String(rpm_filt)+
            ";pwr:"+ String(pwr)+
            ";pwr_filt:"+ String(pwr_filt));            
            
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
    
    
  } else {

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

float RPMtoMpS(float rpm){
  return (2 * PI * (WRAD/1000)) / 60 * rpm;
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