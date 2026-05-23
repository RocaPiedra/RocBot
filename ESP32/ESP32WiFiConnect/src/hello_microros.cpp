/*
 * RocBot micro-ROS Hello World
 *
 * Minimal micro-ROS test for ESP32.
 * Publishes "Hello World" messages and blinks built-in LED
 * to confirm micro-ROS communication is alive.
 *
 * ROS 2 Topics:
 *   Publishers:
 *     - /rocbot/hello (std_msgs/String) — "Hello from ESP32! #<count>"
 *     - /rocbot/heartbeat (std_msgs/UInt32) — alive counter
 *   Subscribers:
 *     - /rocbot/hello/cmd (std_msgs/String) — send "blink" to toggle LED
 *
 * Build:
 *   pio run -e hello_microros -t upload
 *
 * Monitor (optional, for debug prints):
 *   pio device monitor -p /dev/ttyUSB0 -b 115200
 *
 * On the host side, run the micro-ROS agent over UDP:
 *   docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
 *
 * WiFi transport uses board_microros_transport = wifi in platformio.ini.
 * Update AGENT_IP in include/ssid.hpp to match your host PC's IP.
 */

#include <Arduino.h>
#include "ssid.hpp"

// micro-ROS includes (provided by micro_ros_platformio)
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int32.h>

// ============== Configuration ==============
#define PUBLISH_INTERVAL_MS   1000   // Publish hello every 1 second
#define HEARTBEAT_INTERVAL_MS 5000   // Heartbeat every 5 seconds
#define LED_PIN               2      // ESP32 built-in LED
#define AGENT_WAIT_MS         3000   // Time to wait for agent before init

// ============== micro-ROS Handles ==============
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Publishers
rcl_publisher_t pub_hello;
rcl_publisher_t pub_heartbeat;

// Subscribers
rcl_subscription_t sub_cmd;

// Messages
std_msgs__msg__String msg_hello;
std_msgs__msg__UInt32 msg_heartbeat;
std_msgs__msg__String msg_cmd;

// ============== State ==============
uint32_t hello_count = 0;
unsigned long last_hello_time = 0;
unsigned long last_heartbeat_time = 0;
bool led_state = false;
bool microros_connected = false;
int reconnect_attempts = 0;

// String buffers
char hello_buffer[128];
char cmd_buffer[64];

// ============== Error Handling ==============
void error_loop() {
    Serial.println("micro-ROS fatal error! Restarting in 3s...");
    delay(3000);
    ESP.restart();
}

// ============== Cleanup ==============
void destroy_microros_entities() {
    if (microros_connected) {
        (void)rcl_publisher_fini(&pub_hello, &node);
        (void)rcl_publisher_fini(&pub_heartbeat, &node);
        (void)rcl_subscription_fini(&sub_cmd, &node);
        (void)rclc_executor_fini(&executor);
        (void)rcl_node_fini(&node);
        (void)rclc_support_fini(&support);
    }
    microros_connected = false;
}

// ============== Callbacks ==============
void cmd_callback(const void *msg_in) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
    String cmd = String(msg->data.data);
    cmd.trim();
    cmd.toLowerCase();

    Serial.print("[CMD] Received: \"");
    Serial.print(cmd);
    Serial.println("\"");

    if (cmd == "blink") {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state ? HIGH : LOW);
        Serial.print("[CMD] LED -> ");
        Serial.println(led_state ? "ON" : "OFF");
    } else if (cmd == "led_on") {
        led_state = true;
        digitalWrite(LED_PIN, HIGH);
        Serial.println("[CMD] LED -> ON");
    } else if (cmd == "led_off") {
        led_state = false;
        digitalWrite(LED_PIN, LOW);
        Serial.println("[CMD] LED -> OFF");
    } else if (cmd == "reset") {
        hello_count = 0;
        Serial.println("[CMD] Counter reset");
    } else if (cmd == "status") {
        Serial.print("[CMD] Hello count: ");
        Serial.println(hello_count);
        Serial.print("[CMD] LED state: ");
        Serial.println(led_state ? "ON" : "OFF");
        Serial.print("[CMD] Uptime: ");
        Serial.print(millis() / 1000);
        Serial.println("s");
    } else {
        Serial.print("[CMD] Unknown command: ");
        Serial.println(cmd);
    }
}

// ============== micro-ROS Setup ==============
bool setup_microros() {
    // Set up WiFi transport (board_microros_transport = wifi in platformio.ini)
    set_microros_wifi_transports((char*)WIFI_SSID_1, (char*)WIFI_PASSWORD, AGENT_IP_ADDRESS, AGENT_PORT);

    allocator = rcl_get_default_allocator();

    // Wait briefly for agent to be ready
    Serial.println("[ROS] Waiting for agent...");
    delay(AGENT_WAIT_MS);

    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("[ROS] Failed to init support");
        return false;
    }

    // Create node
    ret = rclc_node_init_default(&node, "rocbot_hello_node", "", &support);
    if (ret != RCL_RET_OK) {
        Serial.println("[ROS] Failed to init node");
        rclc_support_fini(&support);
        return false;
    }

    // Init executor: 3 handles = 1 sub + 2 pubs
    rclc_executor_init(&executor, &support.context, 3, &allocator);

    // --- Hello publisher ---
    ret = rclc_publisher_init_default(&pub_hello, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "rocbot/hello");
    if (ret != RCL_RET_OK) {
        Serial.println("[ROS] Failed to init hello publisher");
        goto cleanup;
    }

    // --- Heartbeat publisher ---
    ret = rclc_publisher_init_default(&pub_heartbeat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "rocbot/heartbeat");
    if (ret != RCL_RET_OK) {
        Serial.println("[ROS] Failed to init heartbeat publisher");
        goto cleanup;
    }

    // --- Command subscriber ---
    ret = rclc_subscription_init_default(&sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "rocbot/hello/cmd");
    if (ret != RCL_RET_OK) {
        Serial.println("[ROS] Failed to init command subscriber");
        goto cleanup;
    }

    ret = rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        Serial.println("[ROS] Failed to add subscription");
        goto cleanup;
    }

    // Init string message buffers
    msg_hello.data.data = hello_buffer;
    msg_hello.data.size = 0;
    msg_hello.data.capacity = sizeof(hello_buffer);

    msg_cmd.data.data = cmd_buffer;
    msg_cmd.data.size = 0;
    msg_cmd.data.capacity = sizeof(cmd_buffer);

    msg_heartbeat.data = 0;

    return true;

cleanup:
    destroy_microros_entities();
    return false;
}

// ============== Setup ==============
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  RocBot micro-ROS Hello World");
    Serial.println("========================================");
    Serial.println();

    // Try to connect — agent must be running on host side
    microros_connected = setup_microros();

    if (microros_connected) {
        Serial.println("[ROS] micro-ROS initialized successfully!");
        Serial.println("[ROS] Publishing on:  rocbot/hello, rocbot/heartbeat");
        Serial.println("[ROS] Subscribing on: rocbot/hello/cmd");
    } else {
        Serial.println("[ROS] micro-ROS init FAILED — will retry in loop");
        Serial.println("[ROS] Make sure the agent is running:");
        Serial.println("      docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6");
        Serial.print("[ROS] Target agent: ");
        Serial.print(AGENT_IP);
        Serial.print(":");
        Serial.println(AGENT_PORT);
    }

    Serial.println();
    Serial.println("Commands: send 'blink', 'led_on', 'led_off', 'reset', 'status'");
    Serial.println("          to /rocbot/hello/cmd");
    Serial.println();
}

// ============== Loop ==============
void loop() {
    unsigned long now = millis();

    // If not connected, try to reconnect every 3 seconds
    if (!microros_connected) {
        if (now - last_heartbeat_time >= 3000) {
            last_heartbeat_time = now;
            reconnect_attempts++;
            Serial.print("[ROS] Reconnect attempt #");
            Serial.println(reconnect_attempts);
            microros_connected = setup_microros();
            if (microros_connected) {
                Serial.println("[ROS] Reconnected!");
            }
        }
        return;  // Skip publishing until connected
    }

    // Process micro-ROS (handle incoming subscriptions)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // --- Publish hello message ---
    if (now - last_hello_time >= PUBLISH_INTERVAL_MS) {
        last_hello_time = now;
        hello_count++;
        int len = snprintf(hello_buffer, sizeof(hello_buffer),
            "Hello from ESP32! #%lu (uptime: %lus)",
            hello_count, now / 1000);
        msg_hello.data.size = len;

        rcl_ret_t ret = rcl_publish(&pub_hello, &msg_hello, NULL);
        if (ret == RCL_RET_OK) {
            Serial.print("[ROS] Published: ");
            Serial.println(hello_buffer);
        } else {
            Serial.print("[ROS] Publish failed, rc=");
            Serial.println(ret);
        }
    }

    // --- Publish heartbeat ---
    if (now - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
        last_heartbeat_time = now;
        msg_heartbeat.data = hello_count;
        (void)rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);

        // Blink LED briefly to show we're alive
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        if (!led_state) {
            digitalWrite(LED_PIN, LOW);
        }
    }
}
