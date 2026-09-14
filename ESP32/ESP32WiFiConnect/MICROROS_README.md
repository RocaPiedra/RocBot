# RocBot micro-ROS Setup

## Overview

4-motor omnidirectional ESP32 controller (`main_microros.cpp`) with ROS 2
over micro-ROS WiFi/UDP, plus a serial fallback using the same commands as
`main_debug.cpp`.

Motors: FL `32/35/34/33/25`, FR `14/22/23/27/26`,
RL `13/16/17/4/5`, RR `18/36/39/19/21` (see `AGENTS.md`).

## Files

| File | Description |
|------|-------------|
| `src/main_microros.cpp` | Latest: 4-motor micro-ROS controller (WiFi/UDP) + serial fallback |
| `src/main_debug.cpp` | Serial-only version (same commands, no ROS) — tuning without agent |
| `test/main_serial.cpp` | Legacy serial-only version (archived) |
| `test/hello_microros.cpp` | Minimal micro-ROS transport smoke test (archived) |

## Building

```bash
cd ESP32/ESP32WiFiConnect
pio run -e microros -t upload   # 4-motor micro-ROS controller (latest)
pio run -e debug -t upload      # Serial-only debug version
```

## micro-ROS Agent (on host PC)

The firmware uses WiFi/UDP transport (`board_microros_transport = wifi`).
Set `AGENT_IP` in `include/ssid.hpp` to your PC's IP, then:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

### Verify Connection

```bash
ros2 topic list
# /rocbot/motor_fl/target_rpm  /rocbot/motor_fr/target_rpm
# /rocbot/motor_rl/target_rpm  /rocbot/motor_rr/target_rpm
# /rocbot/motor_fl/target_pwm  ... (direct PWM -255..255 per motor)
# /rocbot/command               (std_msgs/String, same commands as serial)
# /rocbot/motor_fl/rpm  /rocbot/motor_fl/pwm  ... (per motor)
# /rocbot/debug                 (std_msgs/String, mirrors serial debug line)

ros2 topic echo /rocbot/motor_fl/rpm
ros2 topic pub /rocbot/motor_fl/target_rpm std_msgs/Float64 "{data: 60}"
ros2 topic pub /rocbot/command std_msgs/String "{data: 's'}"   # stop all
```

## Serial Fallback (without micro-ROS)

```bash
pio device monitor
# Commands (all 4 motors unless noted):
# g      - Get motor state + PID params
# 60     - Set target RPM (all motors, PID mode)
# d100   - Direct PWM forward 0-255 (all motors)
# D100   - Direct PWM reverse 0-255 (all motors)
# e      - Encoder debug info (all motors)
# p      - Enable PID mode
# s      - Stop
# kp/ki/kd/os<val> - Set PID gains / output scale
# step<val>        - Step test to target RPM
```

Per-motor direct PWM and per-motor RPM targets are available via the
`/rocbot/motor_<fl,fr,rl,rr>/target_pwm` and `.../target_rpm` topics.

## Troubleshooting

### Motors always turn in one direction

1. **Check L298N wiring**:
   - IN1=HIGH, IN2=LOW = Forward
   - IN1=LOW, IN2=HIGH = Reverse
2. **Check encoder direction**:
   - ENCB level on ENCA rising edge determines direction
   - If always incrementing, encoder direction is wrong
3. **Use debug mode**:
   ```bash
   pio run -e debug -t upload
   # Send 'e' to see encoder values
   # Send 'd100' for direct PWM test
   ```

### micro-ROS connection fails

1. Agent must be running first: `docker ... udp4 --port 8888 -v6`
2. `AGENT_IP` in `include/ssid.hpp` must match the PC's LAN IP
3. ESP32 and PC must be on the same 2.4 GHz WiFi network
4. ESP32 prints `[ROS] Retrying micro-ROS connection...` every 3 s until linked

### Performance issues

- Reduce `debugPrintInterval` in code (default 100 ms publishes 9 topics)
- Use filtered RPM values for control
- Ensure 5ms control loop is consistent
