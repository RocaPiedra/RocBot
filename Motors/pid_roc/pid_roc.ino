#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // GREEN
#define PWM 5 // EN-A
#define IN2 6
#define IN1 7
#define MAXCPR 330 //1320 // PULSOS POR REVOLUCIÓN DEL MOTOR
#define WRAD 48 // RADIO EN mm
// #define PI 3.1416
#define MAXRPM 330
#define REFRESHRATE 50 // ms

volatile int pulsos = 0; // specify pulsos as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
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
float rpm=0;

unsigned long temp_ms;
int tiempoCambioVelocidad=5; // seconds
float temporizadorCambioVelocidad=0;

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.begin(115200);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
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
  float temporizadorCambioValor = fabs(((float) (currT - temporizadorCambioVelocidad))/( 1.0e6 ));
  // float pwr;
  
  if (deltaTms >= REFRESHRATE){
      //Modifica las variables de la interrupción forma atómica
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        //rpm = float(pulsos * 60.0 / 374.22); //RPM
        // rpm = float((60.0 * 1000.0 / MAXCPR ) / deltaTs * pulsos);
        // rpm = float(pulsos/MAXCPR) / (60*1.0e3*deltaTms);
        revoluciones = (float)pulsos/(float)MAXCPR;
        rps = revoluciones*1000/deltaTms;
        rpm = rps*60;
        // Serial.println(" - RPM: " + String(rpm) + " - pulsos: " + String(pulsos) + " - revoluciones: " + String(revoluciones) + " - RPS: " + String(rps) + " - deltaTs: " + String(deltaTs) + " - deltaTms: " + String(deltaTms));
        prevT = currT;
        pulsos = 0;
      }
      // rpm = float(pulsos/MAXCPR) * (60/deltaTs);
      // prevT = currT;
      // Serial.println("recalculamos RPM: " + String(rpm) + " RPM - total de pulsos: " + String(pulsos));
      // pulsos = 0;
    // error
    e = rpm - target;

    // derivative
    dedt = (e-eprev)/(deltaTs);

    // integral
    eintegral = eintegral + e*deltaTs;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral; // offset only tunes specific value - no use - 12.5; // introducimos offset

    // motor power
    pwr = fabs(u);
    if( pwr > 255 ){
      pwr = 255;
    }
    // motor direction
    int dir = 1;
    if(u<0){
      dir = -1;
    }

    // test values
    // pwr = 10;

    // signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);
    // store previous error
    eprev = e;
    }    
    // if (deltaTms >= REFRESHRATE){
    //     logger(target, rpm, pwr, e);  
    // }
    
    squaredInputSignal(temporizadorCambioValor, currT);
    // if (fabs(temporizadorCambioValor) >= tiempoCambioVelocidad){
    //     temporizadorCambioVelocidad = currT;
    //     if (target_value <= 60){
    //       target_value = 180;
    //     }else{
    //       target_value = 60;
    //     }
    // }

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

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pulsos++;
  }
  else{
    pulsos--;
  }
}

void readSerialInput(){
  
  if (Serial.available() > 0) {
    
    
    // read the incoming byte:
    float userInput = Serial.read();               // read user input
      
      if(userInput == 'g'){                  // if we get expected value 
   // read the input pin
            Serial.println("target:"+ String(target_value) +";rpm:"+ String(rpm)+";pwr:"+ String(pwr));            
            
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
    if (fabs(elapsed_time) >= tiempoCambioVelocidad){
        temporizadorCambioVelocidad = currT;
        if (target_value <= 60){
          target_value = 180;
        }else{
          target_value = 60;
        }
    }
}

float sinusoidalInputSignal(){

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