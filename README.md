# RocBot
Personal project where I build and program an onmidirectional AMR!

## Motors
I'm using a set of 4 DC motors with encoder, you can find the parameters if the motors in the MotorParameters.xml file.

Currently I'm trying to do precise speed control using an arduino uno.

At Jan 21st a low pass filter added to the speed control (both in RPM calculation and PWR output calculation) has improved the signal as seen in the no_filtering vs filtered graphs. The response is slightly slow, it still needs improving.

# Parts
Arduino Uno
L298N driver
