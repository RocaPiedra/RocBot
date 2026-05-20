/*
 * RocBot ESP32 Motor Controller with micro-ROS
 *
 * Official micro_ros_platformio integration.
 *
 * ROS 2 Topics:
 *   Subscribers:
 *     - /rocbot/motor_fl/target_rpm (std_msgs/Float64)
 *     - /rocbot/motor_fr/target_rpm (std_msgs/Float64)
 *     - /rocbot/command (std_msgs/String)
 *
 *   Publishers:
 *     - /rocbot/motor_fl/state (std_msgs/Float64MultiArray)
 *       [target, rpm, rpm_filt, pwr_filt]
 *     - /rocbot/motor_fr/state (std_msgs/Float64MultiArray)
 *       [target, rpm, rpm_filt, pwr_filt]
 *     - /rocbot/debug (std_msgs/String)
 *       Same format as serial debug output
 *
 * Serial fallback: same commands as main_debug.cpp
 *
 * Build:
 *   pio lib install                  # First time only
 *   pio run -e microros              # Build
 *   pio run -e microros -t upload    # Flash
 *   pio run -e microros --target clean_microros  # Force rebuild
 */

#include <Arduino.h>
#include "MotorController.hpp"
#include "PIDClass.hpp"

// micro-ROS includes (provided by micro_ros_platformio)
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/string.h>

// ============== Motor Configuration ==============
#define MAXCPR 330
#define MAXRPM 330
#define REFRESHRATE 5 // ms
#define NUMMOTORS 2   // FL, FR

// Pin assignments (ESP32) — same as main_debug.cpp
MotorController MotorFL("FL", 32, 35, 34, 33, 25, 1.0, 0.0, 0.0);
MotorController MotorFR("FR", 14, 22, 23, 27, 26, 1.0, 0.0, 0.0);

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

// ============== ISR Handlers ==============
void IRAM_ATTR ISRReadEncoderFR() {
    MotorFR.readEncoder();
    encFR_interrupts++;
}

void IRAM_ATTR ISRReadEncoderFL() {
    MotorFL.readEncoder();
    encFL_interrupts++;
}

// ============== micro-ROS Variables ==============
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Subscribers
rcl_subscription_t sub_fl_target;
rcl_subscription_t sub_fr_target;
rcl_subscription_t sub_command;

// Publishers
rcl_publisher_t pub_fl_state;
rcl_publisher_t pub_fr_state;
rcl_publisher_t pub_debug;

// Messages
std_msgs__msg__Float64MultiArray msg_fl_state;
std_msgs__msg__Float64MultiArray msg_fr_state;
std_msgs__msg__String msg_command;
std_msgs__msg__String msg_debug;
std_msgs__msg__Float64 msg_fl_target;
std_msgs__msg__Float64 msg_fr_target;

// State buffers
double fl_state_data[4];
double fr_state_data[4];
char debug_buffer[512];
char command_buffer[256];

// Incoming ROS targets
float ros_target_fl = 0.0;
float ros_target_fr = 0.0;
bool ros_targets_active = false;

// ============== Serial Input (same as main_debug.cpp) ==============
void readSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) return;

        char firstChar = input.charAt(0);

        if (input.startsWith("kp")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.kp = val;
            Serial.print("Kp set to: "); Serial.println(val);
            return;
        }
        if (input.startsWith("ki")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.ki = val;
            Serial.print("Ki set to: "); Serial.println(val);
            return;
        }
        if (input.startsWith("kd")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.kd = val;
            Serial.print("Kd set to: "); Serial.println(val);
            return;
        }
        if (input.startsWith("os")) {
            float val = input.substring(2).toFloat();
            for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.output_scale = val;
            Serial.print("Output scale set to: "); Serial.println(val);
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
            Serial.print("Step test starting: target "); Serial.println(val);
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
                direct_mode = true; pid_enabled = false; step_test_mode = false;
                reverse_direct = false; direct_pwm = pwm;
                Serial.print("Direct PWM FORWARD: "); Serial.println(direct_pwm);
                break;
            }
            case 'D': {
                int pwm = input.substring(1).toInt();
                pwm = constrain(pwm, 0, 255);
                direct_mode = true; pid_enabled = false; step_test_mode = false;
                reverse_direct = true; direct_pwm = pwm;
                Serial.print("Direct PWM REVERSE: "); Serial.println(direct_pwm);
                break;
            }
            case 'e': {
                Serial.println("--- ENCODER DEBUG ---");
                Serial.print("FR: pulses="); Serial.print(MotorFR.GetPulses());
                Serial.print(", ENCB="); Serial.print(digitalRead(MotorFR.EncBPin));
                Serial.print(", interrupts="); Serial.println(encFR_interrupts);
                Serial.print("FL: pulses="); Serial.print(MotorFL.GetPulses());
                Serial.print(", ENCB="); Serial.print(digitalRead(MotorFL.EncBPin));
                Serial.print(", interrupts="); Serial.println(encFL_interrupts);
                break;
            }
            case 'p': {
                pid_enabled = true; direct_mode = false; step_test_mode = false;
                Serial.println("PID mode enabled");
                Serial.print("Current PID: Kp="); Serial.print(motors[0]->PID.kp);
                Serial.print(" Ki="); Serial.print(motors[0]->PID.ki);
                Serial.print(" Kd="); Serial.println(motors[0]->PID.kd);
                break;
            }
            case 's': {
                pid_enabled = false; direct_mode = false; step_test_mode = false;
                target_value = 0;
                for(int id = 0; id < NUMMOTORS; id++) motors[id]->stop();
                Serial.println("Motors stopped");
                break;
            }
            default: {
                int rpm_val = input.toInt();
                if (input.length() > 0 && (isdigit(input.charAt(0)) || input.charAt(0) == '-')) {
                    rpm_val = constrain(rpm_val, -MAXRPM, MAXRPM);
                    target_value = rpm_val;
                    pid_enabled = true; direct_mode = false; step_test_mode = false;
                    Serial.print("Target RPM set to: "); Serial.println(target_value);
                }
                break;
            }
        }
    }
}

// ============== ROS Subscriber Callbacks ==============
void sub_fl_callback(const void *msg_in) {
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    ros_target_fl = (float)msg->data;
    ros_targets_active = true;
}

void sub_fr_callback(const void *msg_in) {
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    ros_target_fr = (float)msg->data;
    ros_targets_active = true;
}

void sub_command_callback(const void *msg_in) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
    String cmd = String(msg->data.data);
    cmd.trim();

    if (cmd.startsWith("kp")) {
        float val = cmd.substring(2).toFloat();
        for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.kp = val;
    } else if (cmd.startsWith("ki")) {
        float val = cmd.substring(2).toFloat();
        for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.ki = val;
    } else if (cmd.startsWith("kd")) {
        float val = cmd.substring(2).toFloat();
        for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.kd = val;
    } else if (cmd.startsWith("os")) {
        float val = cmd.substring(2).toFloat();
        for (int i = 0; i < NUMMOTORS; i++) motors[i]->PID.output_scale = val;
    } else if (cmd == "p") {
        pid_enabled = true; direct_mode = false; step_test_mode = false;
    } else if (cmd == "s") {
        pid_enabled = false; direct_mode = false; step_test_mode = false;
        target_value = 0;
        for(int id = 0; id < NUMMOTORS; id++) motors[id]->stop();
    } else {
        int rpm_val = cmd.toInt();
        if (cmd.length() > 0 && (isdigit(cmd.charAt(0)) || cmd.charAt(0) == '-')) {
            target_value = constrain(rpm_val, -MAXRPM, MAXRPM);
            pid_enabled = true; direct_mode = false; step_test_mode = false;
        }
    }
}

// ============== Debug Output (same format as main_debug.cpp) ==============
void printDebugInfo() {
    if (millis() - lastDebugPrint < debugPrintInterval) return;
    lastDebugPrint = millis();

    Serial.print("--- DEBUG T:");
    Serial.print(millis());
    Serial.print(" ---");

    if (step_test_mode) {
        Serial.print(" STEP_TEST->"); Serial.print(step_test_target);
    } else if (direct_mode) {
        Serial.print(" DIRECT:");
        Serial.print(reverse_direct ? "REV " : "FWD ");
        Serial.print(direct_pwm);
    } else if (pid_enabled) {
        Serial.print(" PID");
    } else {
        Serial.print(" STOP");
    }

    Serial.print(" Tgt:"); Serial.print(target_value);
    Serial.print(" Kp:"); Serial.print(motors[0]->PID.kp, 2);
    Serial.print(" Ki:"); Serial.print(motors[0]->PID.ki, 2);
    Serial.print(" Kd:"); Serial.print(motors[0]->PID.kd, 2);
    Serial.print(" | ");

    for(int id = 0; id < NUMMOTORS; id++) {
        MotorController* m = motors[id];

        Serial.print(m->motorPos);
        Serial.print(" RPM:"); Serial.print(m->CurrentRPM, 1);
        Serial.print(" F:"); Serial.print(m->FilteredRPM, 1);
        Serial.print(" PWM:"); Serial.print(m->PID.pwr_filt, 1);
        Serial.print(" Out:"); Serial.print(m->PID.pwr, 1);

        int in1 = digitalRead(m->In1Pin);
        int in2 = digitalRead(m->In2Pin);
        Serial.print(" Dir:");
        if (in1 == HIGH && in2 == LOW) Serial.print("FWD");
        else if (in1 == LOW && in2 == HIGH) Serial.print("REV");
        else Serial.print("STP");
        Serial.print(" Pulses:"); Serial.print(String(m->GetPulses()));
        Serial.print(" | ");
    }
    Serial.println();
}

// ============== micro-ROS Setup (Official API) ==============
void setup_micro_ros() {
    // Set up serial transport (official method)
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "rocbot_motor_controller", "", &support);

    // 6 handles: 3 subs + 3 pubs
    rclc_executor_init(&executor, &support.context, 6, &allocator);

    // --- Subscribers ---
    rclc_subscription_init_default(&sub_fl_target, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "rocbot/motor_fl/target_rpm");
    rclc_subscription_init_default(&sub_fr_target, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "rocbot/motor_fr/target_rpm");
    rclc_subscription_init_default(&sub_command, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "rocbot/command");

    rclc_executor_add_subscription(&executor, &sub_fl_target, &msg_fl_target, &sub_fl_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_fr_target, &msg_fr_target, &sub_fr_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_command, &msg_command, &sub_command_callback, ON_NEW_DATA);

    // --- Publishers ---
    rclc_publisher_init_default(&pub_fl_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "rocbot/motor_fl/state");
    rclc_publisher_init_default(&pub_fr_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "rocbot/motor_fr/state");
    rclc_publisher_init_default(&pub_debug, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "rocbot/debug");

    // Init state message buffers
    msg_fl_state.data.data = fl_state_data;
    msg_fl_state.data.size = 4;
    msg_fl_state.data.capacity = 4;
    msg_fr_state.data.data = fr_state_data;
    msg_fr_state.data.size = 4;
    msg_fr_state.data.capacity = 4;

    msg_command.data.data = command_buffer;
    msg_command.data.size = 0;
    msg_command.data.capacity = sizeof(command_buffer);

    msg_debug.data.data = debug_buffer;
    msg_debug.data.size = 0;
    msg_debug.data.capacity = sizeof(debug_buffer);

    Serial.println("micro-ROS initialized!");
}

void publish_motor_states() {
    // FL state: [target, rpm, rpm_filt, pwr_filt]
    int fl_target = ros_targets_active ? (int)ros_target_fl : target_value;
    fl_state_data[0] = fl_target;
    fl_state_data[1] = MotorFL.CurrentRPM;
    fl_state_data[2] = MotorFL.FilteredRPM;
    fl_state_data[3] = MotorFL.PID.pwr_filt;
    msg_fl_state.data.size = 4;

    // FR state: [target, rpm, rpm_filt, pwr_filt]
    int fr_target = ros_targets_active ? (int)ros_target_fr : target_value;
    fr_state_data[0] = fr_target;
    fr_state_data[1] = MotorFR.CurrentRPM;
    fr_state_data[2] = MotorFR.FilteredRPM;
    fr_state_data[3] = MotorFR.PID.pwr_filt;
    msg_fr_state.data.size = 4;

    rcl_publish(&pub_fl_state, &msg_fl_state, NULL);
    rcl_publish(&pub_fr_state, &msg_fr_state, NULL);
}

void publish_debug_string() {
    // Build same format as serial debug line
    int len = 0;
    len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len,
        "--- DEBUG T:%lu ---", millis());

    if (step_test_mode) {
        len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len,
            " STEP_TEST->%d", step_test_target);
    } else if (direct_mode) {
        len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len,
            " DIRECT:%s %d", reverse_direct ? "REV" : "FWD", direct_pwm);
    } else if (pid_enabled) {
        len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len, " PID");
    } else {
        len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len, " STOP");
    }

    len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len,
        " Tgt:%d Kp:%.2f Ki:%.2f Kd:%.2f | ",
        target_value, motors[0]->PID.kp, motors[0]->PID.ki, motors[0]->PID.kd);

    for(int id = 0; id < NUMMOTORS; id++) {
        MotorController* m = motors[id];
        int in1 = digitalRead(m->In1Pin);
        int in2 = digitalRead(m->In2Pin);
        const char* dir = (in1 == HIGH && in2 == LOW) ? "FWD" :
                          (in1 == LOW && in2 == HIGH) ? "REV" : "STP";

        len += snprintf(debug_buffer + len, sizeof(debug_buffer) - len,
            "%s RPM:%.1f F:%.1f PWM:%.1f Out:%.1f Dir:%s Pulses:%d | ",
            m->motorPos.c_str(), m->CurrentRPM, m->FilteredRPM,
            m->PID.pwr_filt, m->PID.pwr, dir, m->GetPulses());
    }

    msg_debug.data.size = len;
    rcl_publish(&pub_debug, &msg_debug, NULL);
}

// ============== Main Setup ==============
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("=== RocBot Motor Controller with micro-ROS ===");
    Serial.println("PID Tuning commands: kp<val> ki<val> kd<val> os<val>");
    Serial.println("Step test: step<target>");

    attachInterrupt(digitalPinToInterrupt(MotorFR.EncAPin), ISRReadEncoderFR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorFL.EncAPin), ISRReadEncoderFL, RISING);

    Serial.println("Encoder interrupts attached");

    // Initialize micro-ROS (official API)
    setup_micro_ros();

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

// ============== Main Loop ==============
void loop() {
    // Handle serial input (fallback control)
    readSerialInput();

    // Process micro-ROS executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Step test mode - change target after delay
    if (step_test_mode && millis() - step_test_start > 5000) {
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

                // Use ROS target if active, otherwise serial target
                int actual_target = ros_targets_active ?
                    (id == 0 ? (int)ros_target_fl : (int)ros_target_fr) :
                    (step_test_mode ? step_test_target : target_value);

                motors[id]->controlMotor(actual_target, deltaTms * 1000);
            }
        }
    }

    // Debug output (serial)
    printDebugInfo();

    // Publish ROS state at debug interval rate
    if (millis() - lastDebugPrint < debugPrintInterval + 5) {
        publish_motor_states();
        publish_debug_string();
    }
}
