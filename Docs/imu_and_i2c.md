# IMU and I2C Communication

## IMU: BNO080 / BNO085 (GY-BNO080)

A 9-DOF absolute orientation IMU connected via I2C.

### Specifications

- **Sensor**: BNO080 (or BNO085) from CEVA/Hillcrest Labs
- **Board**: GY-BNO080
- **Interface**: I2C
- **I2C Address**: 0x4B (75 decimal)
- **Voltage**: 3.3V
- **Microcontroller**: Arduino Mega 2560 Pro Mini (for memory requirements)

### Wiring

| IMU Pin | Mega 2560 Pro Mini |
|---------|-------------------|
| SCL     | D20 (SCL)         |
| SDA     | D21 (SDA)         |
| VCC     | 3.3V              |
| GND     | GND               |

### Firmware

Two approaches have been tested:

#### 1. Adafruit BNO08x Library (`adafruit_demo/`)

Uses Adafruit's BNO08x library (requires Arduino Mega or similar for memory).

- Enables `SH2_GAME_ROTATION_VECTOR` report
- Outputs quaternion (r, i, j, k) in the main loop
- Handles sensor reset detection

#### 2. Raw I2C Reading (`imu_reader/`)

Attempts direct register reads via the Wire library.

- Requests 86 bytes from the IMU
- Falls back to I2C address scanning if default fails
- Useful for debugging and understanding the raw protocol

## I2C Protocol Experiments

Located in `I2C/` — basic experiments to validate I2C communication between Arduinos:

### `i2c_scanner/`
Standard I2C bus scanner, scans addresses 0–130, reports found devices.

### `arduino_i2c_master/`
Transmits incrementing values (0–5) to a slave device every 2 seconds. Scans for devices and stores the first found address.

### `arduino_i2c_slave/`
Listens on address 9, receives integer values, blinks LED on pin 13 proportional to received value (100ms × value).

## Future Work

- Integrate IMU orientation with motor control for closed-loop heading
- Implement sensor fusion (complementary or Madgwick filter) if raw data is used
- Use IMU data for odometry correction
