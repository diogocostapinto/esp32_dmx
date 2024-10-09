#include <Arduino.h>
#include <esp_dmx.h>
#include <ArtnetWifi.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

const int transmitPin = 17;
const int receivePin = 16;
const int enablePin = 21;
const dmx_port_t dmxPort = 1;
int universe = 0;  
const int builtInLedPin = 2;

byte data[DMX_PACKET_SIZE];

ArtnetWifi artnet;
WebServer server(80);

const char* defaultSSID = "Apto";
const char* defaultPassword = "password";
String ssid = defaultSSID;
String password = defaultPassword;

const int ssidAddr = 0;
const int passwordAddr = 32; 

bool wifiConnected = false;
unsigned long lastDmxSendTime = 0;
const int dmxSendInterval = 25;
const int apModeBlinkInterval = 2000;

void saveWiFiCredentials(const String& ssid, const String& password) {
  EEPROM.writeString(ssidAddr, ssid);
  EEPROM.writeString(passwordAddr, password);
  EEPROM.commit();
}

void loadWiFiCredentials() {
  ssid = EEPROM.readString(ssidAddr);
  password = EEPROM.readString(passwordAddr);
}

void setupDualWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("_Dmx11", "00000000");
  Serial.print("AP Mode IP Address: ");
  Serial.println(WiFi.softAPIP());

  WiFi.begin(ssid.c_str(), password.c_str());
  int retryCount = 0;

  while (WiFi.status() != WL_CONNECTED && retryCount < 30) {
    delay(500);
    retryCount++;
    digitalWrite(builtInLedPin, retryCount % 2);
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  Serial.println(wifiConnected ? "Connected to WiFi" : "Failed to connect, AP mode only.");
  digitalWrite(builtInLedPin, wifiConnected ? LOW : HIGH);

  if (wifiConnected) {
    Serial.print("STA Mode IP Address: ");
    Serial.println(WiFi.localIP());
    MDNS.begin("dmx11");
  }
}

void setupWebServer() {
  server.on("/", [ssid, password, universe]() mutable {
    String html = "<html><body>"
                  "<h1>Quanta DMX Box 11</h1>"
                  "<p>AP IP Address: " + WiFi.softAPIP().toString() + "</p>"
                  "<p>STA IP Address: " + WiFi.localIP().toString() + "</p>"
                  "<form action='/setwifi' method='POST'>"
                  "SSID:<input type='text' name='ssid' value='" + ssid + "'><br>"
                  "Password:<input type='password' name='password' value='" + password + "'><br>"
                  "DMX Universe:<input type='number' name='universe' value='" + String(universe) + "'><br>"
                  "<input type='submit' value='Update'>"
                  "</form></body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/setwifi", [ssid, password, universe]() mutable {
    ssid = server.arg("ssid");
    password = server.arg("password");
    universe = server.arg("universe").toInt();
    saveWiFiCredentials(ssid, password);
    setupDualWiFi();
    server.send(200, "text/plain", "Settings updated. Please reconnect.");
  });

  server.begin();
}

void setupDMX() {
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmxPort, &config, NULL, 0);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);
}

void setupArtNet() {
  artnet.begin();
  artnet.setArtDmxCallback([](uint16_t receivedUniverse, uint16_t length, uint8_t sequence, uint8_t* receivedData) {
    if (receivedUniverse == universe && length <= DMX_PACKET_SIZE) {
      memcpy(&data[1], receivedData, length);
      dmx_write(dmxPort, data, length + 1);
    }
  });
}

void blinkLed(bool state) {
  digitalWrite(builtInLedPin, state);
}

void loop() {
  artnet.read();
  server.handleClient();

  if (millis() - lastDmxSendTime >= dmxSendInterval) {
    dmx_send(dmxPort);
    lastDmxSendTime = millis();
  }

  blinkLed(!wifiConnected && (millis() % apModeBlinkInterval < apModeBlinkInterval / 2));
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  loadWiFiCredentials();
  pinMode(builtInLedPin, OUTPUT);
  setupDualWiFi();
  setupWebServer();
  setupDMX();
  setupArtNet();
}
