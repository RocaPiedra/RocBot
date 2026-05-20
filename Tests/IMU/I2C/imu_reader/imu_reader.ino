#include <Wire.h>

byte device_address = 74; //75
int x = 0;
int error = -1;


void setup()
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("\nI2C IMU Reader");
    Wire.beginTransmission(device_address);
    error = Wire.endTransmission();
  
    if (error == 0){
        Serial.println("Device connected");
    } else if (error == 4){
        Serial.println("Default device address ("+ String(device_address) + ") failed to connect. Performing scan...");
        device_address = 0;
    } else {
        Serial.println("Uknown code error: " + String(error));
        device_address = 0;
    }
}


void loop(){

  if (device_address != 0){
    Wire.requestFrom(device_address, 86); // Request 6 bytes of data from Slave with address 8

  while (Wire.available()) {
    byte c = Wire.read();
    Serial.print(c);
  }

  delay(1000); // Adjust the delay based on your requirements
  } else {
    device_address = find_i2c_address();
  }
}

byte find_i2c_address(){

  byte error, address;
  int nDevices;
  bool deviceFound = false;

  Serial.println("Scanning...");

  for(address = 0; address <= 130; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      Serial.println("Starting communication with address: " + String(device_address) + "!");
      return address;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
    Serial.println("No I2C devices found\n");
  return 0;
}