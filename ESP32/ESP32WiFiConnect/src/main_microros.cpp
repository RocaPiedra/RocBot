/*
 * RocBot ESP32 Motor Controller with micro-ROS
 * 
 * This version integrates with ROS 2 via micro-ROS agent.
 * 
 * ROS Topics:
 *   Subscribers:
 *     - /motor_fl/target_rpm (Float64)
 *     - /motor_fr/target_rpm (Float64)
 *     - /motor_bl/target_rpm (Float64)
 *     - /motor_br/target_rpm (Float64)
 *   
 *   Publishers:
 *     - /joint_states (sensor_msgs/JointState)
 *     - /motor_fl/state (Float64)
 *     - /motor_fr/state (Float64)
 *     - /motor_bl/state (Float64)
 *     - /motor_br/state (Float64)
 * 
 * Build: platformio run -e microros -t upload
 */

#include <Arduino.h>
#include "MotorController.hpp"
#include "PIDClass.hpp"

// micro-ROS includes
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/joint_state.h>

// ============== Motor Configuration ==============
#define MAXCPR 330
#define MAXRPM 330
#define REFRESHRATE 5 // ms
#define NUMMOTORS 2   // Only 2 motors for now: FL, FR

// Pin assignments (ESP32)
MotorController MotorFL("FL", 32, 35, 34, 33, 25, 0.8, 1.0, 0.1);
MotorController MotorFR("FR", 14, 22, 23, 27, 26, 0.8, 1.0, 0.1);

// Array of all motors
MotorController* motors[NUMMOTORS] = {&MotorFL, &MotorFR};
const char* motor_names[NUMMOTORS] = {"fl_joint", "fr_joint"};
int motor_targets[NUMMOTORS] = {0, 0};

long prevT[NUMMOTORS] = {0, 0};

// Timing
unsigned long last_joint_state_pub = 0;
unsigned long last_debug_print = 0;
unsigned long last_log_print = 0;
const unsigned long JOINT_STATE_INTERVAL = 20; // ms (50 Hz)
const unsigned long DEBUG_PRINT_INTERVAL = 100; // ms
const unsigned long LOG_INTERVAL = 50; // ms

// ============== micro-ROS Variables ==============
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Publishers
rcl_publisher_t joint_state_publisher;

// Messages
sensor_msgs__msg__JointState joint_state_msg;

// ============== ISR Handlers ==============
void IRAM_ATTR ISRReadEncoderFR() { MotorFR.readEncoder(); }
void IRAM_ATTR ISRReadEncoderFL() { MotorFL.readEncoder(); }

// ============== Subscriber Callbacks ==============
// Using global variables to store incoming data
float target_fl = 0.0;
float target_fr = 0.0;

void sub_fl_callback(const void *msg_in) {
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    target_fl = msg->data;
    motor_targets[0] = (int)target_fl;
}

void sub_fr_callback(const void *msg_in) {
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msg_in;
    target_fr = msg->data;
    motor_targets[1] = (int)target_fr;
}

// ============== Serial Input ==============
void readSerialInput() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        if (cmd == 'g') {
            // Get motor state
            for(int id = 0; id < NUMMOTORS; id++) {
                Serial.println(motors[id]->GetMotorState());
            }
        } else if (cmd >= '0' && cmd <= '9' || cmd == '-') {
            // Set all motors to target RPM
            int rpm = Serial.parseInt();
            Serial.print("Setting all motors to RPM: ");
            Serial.println(rpm);
            for(int i = 0; i < NUMMOTORS; i++) {
                motor_targets[i] = rpm;
            }
        }
    }
}

// ============== micro-ROS Setup ==============
void setup_micro_ros() {
    // Set up serial transport (USB serial)
    set_microros_serial_transports(115200, "/dev/ttyUSB1");
    
    allocator = rcl_get_default_allocator();
    
    // Create init options
    rclc_support_init(&support, 0, NULL, &allocator);
    
    // Create node
    rclc_node_init_default(&node, "rocbot_motor_controller", "", &support);
    
    // Create executor with 3 handles (2 subs + timer)
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    
    // --- Joint State Publisher ---
    // Initialize message - using static allocation
    joint_state_msg.position.size = NUMMOTORS;
    joint_state_msg.velocity.size = NUMMOTORS;
    joint_state_msg.effort.size = NUMMOTORS;
    joint_state_msg.name.size = NUMMOTORS;
    
    // Use static buffers (micro-ROS requirement)
    static double position_buffer[2];
    static double velocity_buffer[2];
    static double effort_buffer[2];
    static char name_buffer[2][20];
    
    joint_state_msg.position.data = position_buffer;
    joint_state_msg.velocity.data = velocity_buffer;
    joint_state_msg.effort.data = effort_buffer;
    
    for(int i = 0; i < NUMMOTORS; i++) {
        strncpy(name_buffer[i], motor_names[i], 19);
        name_buffer[i][19] = '\0';
    }
    joint_state_msg.name.data = name_buffer;
    
    rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"
    );
    
    // --- Subscribers ---
    rcl_subscription_t sub_fl, sub_fr;
    std_msgs__msg__Float64 msg_fl, msg_fr;
    
    rclc_subscription_init_default(&sub_fl, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "motor_fl/target_rpm");
    rclc_subscription_init_default(&sub_fr, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "motor_fr/target_rpm");
    
    rclc_executor_add_subscription(&executor, &sub_fl, &msg_fl, &sub_fl_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_fr, &msg_fr, &sub_fr_callback, ON_NEW_DATA);
    
    Serial.println("micro-ROS initialized!");
}

void publish_joint_state() {
    // Update joint state message
    for(int i = 0; i < NUMMOTORS; i++) {
        joint_state_msg.velocity.data[i] = motors[i]->FilteredRPM;
        joint_state_msg.effort.data[i] = motors[i]->PID.pwr_filt;
    }
    
    // Get current time
    joint_state_msg.header.stamp.sec = millis() / 1000;
    joint_state_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    
    rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

void print_debug_info() {
    if (millis() - last_debug_print < DEBUG_PRINT_INTERVAL) return;
    last_debug_print = millis();
    
    Serial.println("=== Motor States ===");
    for(int i = 0; i < NUMMOTORS; i++) {
        Serial.printf("%s: Target=%d RPM=%.1f Filt=%.1f PWM=%.1f Dir=%s\n",
            motor_names[i],
            motor_targets[i],
            motors[i]->CurrentRPM,
            motors[i]->FilteredRPM,
            motors[i]->PID.pwr_filt,
            motors[i]->currentDirection ? "FWD" : "REV"
        );
    }
}

void log_motor_data() {
    if (millis() - last_log_print < LOG_INTERVAL) return;
    last_log_print = millis();
    
    // Log to serial in CSV format for Python visualizer
    Serial.print("LOG:");
    for(int i = 0; i < NUMMOTORS; i++) {
        Serial.printf("&%s;target:%d;rpm:%.1f;rpm_filt:%.1f;pwr:%.1f;pwr_filt:%.1f;",
            motor_names[i],
            motor_targets[i],
            motors[i]->CurrentRPM,
            motors[i]->FilteredRPM,
            motors[i]->currentDir * motors[i]->PID.pwr,
            motors[i]->currentDir * motors[i]->PID.pwr_filt
        );
    }
    Serial.println();
}

// ============== Main Setup ==============
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== RocBot Motor Controller with micro-ROS ===");
    
    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(MotorFR.EncAPin), ISRReadEncoderFR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorFL.EncAPin), ISRReadEncoderFL, RISING);
    
    Serial.println("Encoder interrupts attached");
    
    // Initialize micro-ROS
    setup_micro_ros();
    
    Serial.println("\nWaiting for micro-ROS agent connection...");
    Serial.println("Connect agent with: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200");
}

// ============== Main Loop ==============
void loop() {
    // Handle serial input (fallback control)
    readSerialInput();
    
    // Process micro-ROS executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    // Motor control loop
    long currT = micros();
    
    for(int id = 0; id < NUMMOTORS; id++) {
        float deltaTms = ((float)(currT - prevT[id])) / 1.0e3;
        
        if (deltaTms >= REFRESHRATE) {
            // Update RPM from encoder
            motors[id]->updateRPM((float)MAXCPR, deltaTms);
            prevT[id] = currT;
            
            // Control motor with PID
            motors[id]->controlMotor(motor_targets[id], deltaTms * 1000);
        }
    }
    
    // Publish joint states at 50 Hz
    if (millis() - last_joint_state_pub >= JOINT_STATE_INTERVAL) {
        publish_joint_state();
        last_joint_state_pub = millis();
    }
    
    // Debug output
    print_debug_info();
    
    // Log data for visualizer
    log_motor_data();
}