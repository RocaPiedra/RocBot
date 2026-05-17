/*
 * RocBot ESP32 Debug Motor Controller v4
 * 
 * Features for PID tuning:
 * - Dynamic PID parameter updates via serial
 * - Step response test mode
 * - Better debug output
 * 
 * Commands:
 * d<0-255> - Direct forward PWM
 * D<0-255> - Direct reverse PWM  
 * e         - Encoder debug
 * p         - Enable PID mode
 * s         - Stop motors
 * g         - Get motor state
 * <num>     - Set target RPM
 * 
 * PID Tuning commands:
 * kp<value> - Set Kp (e.g., kp0.5)
 * ki<value> - Set Ki (e.g., ki0.1)
 * kd<value> - Set Kd (e.g., kd0.05)
 * t<value>  - Set target RPM
 * step<val> - Run step test to target RPM
 */

#include <Arduino.h>
#include "MotorController.hpp"
#include "PIDClass.hpp"

#define MAXCPR 330
#define MAXRPM 330
#define REFRESHRATE 5 // ms
#define NUMMOTORS 2

// Pin assignments
MotorController MotorFR("FR", 14, 22, 23, 27, 26, 1.0, 0.0, 0.0);
MotorController MotorFL("FL", 32, 35, 34, 33, 25, 1.0, 0.0, 0.0);
MotorController* motors[NUMMOTORS] = {&MotorFL, &MotorFR};

long prevT[NUMMOTORS] = {0, 0};
int target_value = 0;
bool pid_enabled = false;
bool direct_mode = false;
int direct_pwm = 0;
bool reverse_direct = false;

// Test mode
bool step_test_mode = false;
unsigned long step_test_start = 0;
int step_test_target = 0;

unsigned long lastDebugPrint = 0;
unsigned long debugPrintInterval = 100;

volatile unsigned long encFR_interrupts = 0;
volatile unsigned long encFL_interrupts = 0;

void IRAM_ATTR ISRReadEncoderFR() {
    MotorFR.readEncoder();
    encFR_interrupts++;
}

void IRAM_ATTR ISRReadEncoderFL() {
    MotorFL.readEncoder();
    encFL_interrupts++;
}

void readSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 0) return;
        
        char firstChar = input.charAt(0);
        
        // Check for parameter setting (kp, ki, kd, os commands)
        if (input.startsWith("kp")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) {
                motors[i]->PID.kp = val;
            }
            Serial.print("Kp set to: ");
            Serial.println(val);
            return;
        }
        if (input.startsWith("ki")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) {
                motors[i]->PID.ki = val;
            }
            Serial.print("Ki set to: ");
            Serial.println(val);
            return;
        }
        if (input.startsWith("kd")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) {
                motors[i]->PID.kd = val;
            }
            Serial.print("Kd set to: ");
            Serial.println(val);
            return;
        }
        if (input.startsWith("os")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) {
                motors[i]->PID.output_scale = val;
            }
            Serial.print("Output scale set to: ");
            Serial.println(val);
            Serial.println("This multiplies PID output to convert to PWM range");
            return;
        }
        if (input.startsWith("step")) {
            int val = input.substring(4).toInt();
            step_test_mode = true;
            step_test_target = val;
            step_test_start = millis();
            target_value = 0;
            pid_enabled = true;
            direct_mode = false;
            Serial.print("Step test starting: target ");
            Serial.println(val);
            return;
        }
        
        switch(firstChar) {
            case 'g': {
                Serial.println("--- MOTOR STATE ---");
                Serial.print("Kp="); Serial.print(motors[0]->PID.kp);
                Serial.print(" Ki="); Serial.print(motors[0]->PID.ki);
                Serial.print(" Kd="); Serial.println(motors[0]->PID.kd);
                for(int id = 0; id < NUMMOTORS; id++) {
                    Serial.println(motors[id]->GetMotorState());
                }
                break;
            }
            case 'd': {
                int pwm = input.substring(1).toInt();
                pwm = constrain(pwm, 0, 255);
                direct_mode = true;
                pid_enabled = false;
                step_test_mode = false;
                reverse_direct = false;
                direct_pwm = pwm;
                Serial.print("Direct PWM FORWARD: ");
                Serial.println(direct_pwm);
                break;
            }
            case 'D': {
                int pwm = input.substring(1).toInt();
                pwm = constrain(pwm, 0, 255);
                direct_mode = true;
                pid_enabled = false;
                step_test_mode = false;
                reverse_direct = true;
                direct_pwm = pwm;
                Serial.print("Direct PWM REVERSE: ");
                Serial.println(direct_pwm);
                break;
            }
            case 'e': {
                Serial.println("--- ENCODER DEBUG ---");
                Serial.print("FR: pulses=");
                Serial.print(MotorFR.GetPulses());
                Serial.print(", ENCB=");
                Serial.print(digitalRead(MotorFR.EncBPin));
                Serial.print(", interrupts=");
                Serial.println(encFR_interrupts);
                
                Serial.print("FL: pulses=");
                Serial.print(MotorFL.GetPulses());
                Serial.print(", ENCB=");
                Serial.print(digitalRead(MotorFL.EncBPin));
                Serial.print(", interrupts=");
                Serial.println(encFL_interrupts);
                break;
            }
            case 'p': {
                pid_enabled = true;
                direct_mode = false;
                step_test_mode = false;
                Serial.println("PID mode enabled");
                Serial.print("Current PID: Kp=");
                Serial.print(motors[0]->PID.kp);
                Serial.print(" Ki=");
                Serial.print(motors[0]->PID.ki);
                Serial.print(" Kd=");
                Serial.println(motors[0]->PID.kd);
                break;
            }
            case 's': {
                pid_enabled = false;
                direct_mode = false;
                step_test_mode = false;
                target_value = 0;
                for(int id = 0; id < NUMMOTORS; id++) {
                    motors[id]->stop();
                }
                Serial.println("Motors stopped");
                break;
            }
            default: {
                int rpm_val = input.toInt();
                if (input.length() > 0 && (isdigit(input.charAt(0)) || input.charAt(0) == '-')) {
                    rpm_val = constrain(rpm_val, -MAXRPM, MAXRPM);
                    target_value = rpm_val;
                    pid_enabled = true;
                    direct_mode = false;
                    step_test_mode = false;
                    Serial.print("Target RPM set to: ");
                    Serial.println(target_value);
                }
                break;
            }
        }
    }
}

void printDebugInfo() {
    if (millis() - lastDebugPrint < debugPrintInterval) return;
    lastDebugPrint = millis();
    
    Serial.print("--- DEBUG T:");
    Serial.print(millis());
    Serial.print(" ---");
    
    if (step_test_mode) {
        Serial.print(" STEP_TEST->");
        Serial.print(step_test_target);
    } else if (direct_mode) {
        Serial.print(" DIRECT:");
        Serial.print(reverse_direct ? "REV " : "FWD ");
        Serial.print(direct_pwm);
    } else if (pid_enabled) {
        Serial.print(" PID");
    } else {
        Serial.print(" STOP");
    }
    
    Serial.print(" Tgt:");
    Serial.print(target_value);
    Serial.print(" Kp:");
    Serial.print(motors[0]->PID.kp, 2);
    Serial.print(" Ki:");
    Serial.print(motors[0]->PID.ki, 2);
    Serial.print(" Kd:");
    Serial.print(motors[0]->PID.kd, 2);
    Serial.print(" | ");
    
    // Print motor states in compact CSV-like format
    for(int id = 0; id < NUMMOTORS; id++) {
        MotorController* m = motors[id];
        
        Serial.print(m->motorPos);
        Serial.print(" RPM:");
        Serial.print(m->CurrentRPM, 1);
        Serial.print(" F:");
        Serial.print(m->FilteredRPM, 1);
        Serial.print(" PWM:");
        Serial.print(m->PID.pwr_filt, 1);
        Serial.print(" Out:");
        Serial.print(m->PID.pwr, 1);
        
        int in1 = digitalRead(m->In1Pin);
        int in2 = digitalRead(m->In2Pin);
        Serial.print(" Dir:");
        if (in1 == HIGH && in2 == LOW) Serial.print("FWD");
        else if (in1 == LOW && in2 == HIGH) Serial.print("REV");
        else Serial.print("STP");
        Serial.print(" Pulses:");
        Serial.print(String(m->GetPulses()));
        Serial.print(" | ");
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("=== RocBot Debug Motor Controller v4 ===");
    Serial.println("PID Tuning commands: kp<val> ki<val> kd<val>");
    Serial.println("Step test: step<target>");
    
    attachInterrupt(digitalPinToInterrupt(MotorFR.EncAPin), ISRReadEncoderFR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorFL.EncAPin), ISRReadEncoderFL, RISING);
    
    Serial.println("Encoder interrupts attached");
    
    Serial.println("\n=== Commands ===");
    Serial.println("d<0-255>  - Direct forward");
    Serial.println("D<0-255>  - Direct reverse");
    Serial.println("e         - Encoder debug");
    Serial.println("p         - Enable PID");
    Serial.println("s         - Stop");
    Serial.println("g         - Get state + PID params");
    Serial.println("<num>     - Set target RPM");
    Serial.println("kp<val>   - Set Kp");
    Serial.println("ki<val>   - Set Ki");
    Serial.println("kd<val>   - Set Kd");
    Serial.println("os<val>   - Set output scale (default 10)");
    Serial.println("step<val> - Step test");
    
    Serial.println("\n=== Ready ===");
}

void loop() {
    readSerialInput();
    
    // Step test mode - change target after delay
    if (step_test_mode && millis() - step_test_start > 5000) {
        // Test complete, stop
        step_test_mode = false;
        target_value = 0;
        Serial.println("Step test complete");
    }
    
    if (direct_mode) {
        for(int id = 0; id < NUMMOTORS; id++) {
            motors[id]->SetDirection(!reverse_direct);
            motors[id]->SetSpeed(direct_pwm);
        }
    } else if (pid_enabled || step_test_mode) {
        long currT = micros();
        
        for(int id = 0; id < NUMMOTORS; id++) {
            float deltaTms = ((float)(currT - prevT[id])) / 1.0e3;
            
            if (deltaTms >= REFRESHRATE) {
                motors[id]->updateRPM((float)MAXCPR, deltaTms);
                prevT[id] = currT;
                
                // In step test mode, target is step_test_target
                int actual_target = step_test_mode ? step_test_target : target_value;
                motors[id]->controlMotor(actual_target, deltaTms * 1000);
            }
        }
    }
    
    printDebugInfo();
}