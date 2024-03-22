#include <Arduino.h>
#include <esp_dmx.h>
#include <ArtnetWifi.h>
#include <WiFi.h>

// DMX Pins
const int transmitPin = 17;
const int receivePin = 16;
const int enablePin = 21;
const dmx_port_t dmxPort = 1; // Direct assignment for the dmx port

ArtnetWifi artnet;
const int universe = 0;

const char* ssid = "Quanta-Broadcast";
const char* password = "Quanta@2023!";
IPAddress artnetIp(10, 1, 9, 241);
IPAddress routerIP(10, 1, 8, 1);
IPAddress subnetIP(255, 255, 254, 0);

byte data[DMX_PACKET_SIZE];
unsigned long lastUpdate = millis();

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  WiFi.config(artnetIp, routerIP, subnetIP);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void setupDMX() {
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmxPort, &config, NULL, 0);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
}

void setupArtNet() {
  artnet.begin();
  artnet.setArtDmxCallback([](uint16_t receivedUniverse, uint16_t length, uint8_t sequence, uint8_t* receivedData) {
    if (receivedUniverse == universe) {
      if (length <= DMX_PACKET_SIZE) {
        memcpy(&data[1], receivedData, length); // Offset by 1 to align with DMX's 1-indexing
        dmx_write(dmxPort, data, length + 1);
      }
    }
  });
}

void loop() {
  artnet.read();

  static unsigned long lastDmxSendTime = 0;
  if (millis() - lastDmxSendTime >= 25) {
    dmx_send(dmxPort); // Assuming this function exists for simplicity, replace with actual DMX send function if different
    lastDmxSendTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  setupDMX();
  setupArtNet();
}