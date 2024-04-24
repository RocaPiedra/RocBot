# RocBot
A personal project where I build and program an omnidirectional AMR!

## Motors

### Basic Info
I'm using a set of 4 DC motors with an encoder, you can find the parameters of the motors in the MotorParameters.xml file.

Currently, I'm trying to do precise speed control using an Arduino Uno.

On Jan 21st a low pass filter added to the speed control (both in RPM calculation and PWR output calculation) has improved the signal as seen in the no_filtering vs filtered graphs. The response is slightly slow, it still needs improving.

### PID Calibration
Jan 22nd: Kp = 0.8, Kd = 0.1, Ki = 1.0
The response is slow and barely keeps up with fast-paced changes in target RPM. The signal has little jitter, for now, good enough. The signal does not seem to have an offset error at all and can converge to the target RPM. Increasing the proportional value for a faster response should help if I manage to balance it with the derivative term.

## IMU
Currently, I have a GY-BNO080 9 DOF IMU that I bought for 15€, my objective is to read data through the Arduino board using i2c. I've never used this protocol before but it seems quite useful for a 2-pin protocol.

The IMU datasheet says it runs at 3.3 V so I've connected to the 3.3 V output of the Arduino. SCL goes to A4 for the clock signal and SDA goes to A5 for the data signal. Through i2c scanning, I found the IMU's address to be 0x4B - 75

Had to change the microcontroller to take advantage of Adafruit's great library, currently using it to interface with the IMU and Arduino Mega2560 Pro Mini (that can actually fit the size of the library). 

For Mega SCL is in D20 and SDA is in D21. 


# Parts

Microcontrollers:
- Arduino Uno
- Arduino Mega2560 Pro Mini
- ESP32

Drivers:
- L298N driver

IMU: 
- BNO085 (9DOF)
