#include <Wire.h>

byte device_address = 0;
int x = 0;
int error = -1;


void setup()
{
  Wire.begin();

  Serial.begin(9600);
  Serial.println("\nI2C Scanner");
  
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
      device_address = address;
      Serial.println("Starting communication with address: " + String(device_address) + "!");
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (int(device_address) == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(1000);
}


void loop(){

  if (device_address != 0){
    Wire.beginTransmission(device_address); // transmit to device #9
    Serial.println("Sending: " + String(x));
    Wire.write(x);              // sends x 
    error = Wire.endTransmission();    // stop transmitting
    if (error == 0)
    {
      Serial.println(" I2C correct communication established");

    }
    else if (error==4)
    {
      Serial.println(" I2C communication failed");
    }  
    x++; // Increment x
    if (x > 5) x = 0; // `reset x once it gets 6
    delay(2000);
  }
}