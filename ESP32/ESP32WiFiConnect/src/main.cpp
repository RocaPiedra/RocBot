#include <Arduino.h>
#include <Wifi.h>
#include <WifiMulti.h>

#define WIFI_SSID_1 "DIGIFIBRA-PLUS-uC4f"
#define WIFI_SSID_2 "DIGIFIBRA-uC4f"
#define WIFI_PASSWORD "yDukUYNFDT"

WiFiMulti myWiFi;

void setup() {
  Serial.begin(921600);
  // pinMode(LED_BUILTIN, OUTPUT); // STATUS INDICATOR
  myWiFi.addAP(WIFI_SSID_1, WIFI_PASSWORD);
  myWiFi.addAP(WIFI_SSID_2, WIFI_PASSWORD);
  while (myWiFi.run() != WL_CONNECTED){
    delay(100);
  }
  Serial.println("connected to " + String(WiFi.SSID()));
}

void loop() {
  // digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
  Serial.println("alive: connected to " + String(WiFi.SSID()));
    delay(1000);
}
