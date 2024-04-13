#include <Wire.h>
#define I2C_ADD 9
#define LED_PIN 13
int x = 0;
void setup() {
  // Define the LED pin as Output
  pinMode (LED_PIN, OUTPUT);
  // Start the I2C Bus as Slave on address 9
  Wire.begin(I2C_ADD);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
 
  Serial.begin(115200);
  Serial.println("Starting i2c receiver ");
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
    digitalWrite(LED_PIN, LOW);
}
void receiveEvent(int bytes) {
  x = Wire.read();    // read one character from the I2C
  Serial.println("Received: " + String(x));
}
void loop() {
  //If value received is 0 blink LED for 200 ms
  if (x != 0){
    digitalWrite(LED_PIN, HIGH);
    delay(100*x);
    digitalWrite(LED_PIN, LOW);
    delay(100*x);
  }
}