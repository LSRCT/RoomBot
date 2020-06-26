#include <ESP8266WiFi.h>
#include <HCSR04.h>

// PIN constants to avoid confusion
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10 = 1;



// Wifi variables
const char* ssid     = "LSRCT";
const char* password = "83067046472468411597";
const char* host = "192.168.178.20";
const uint16_t port = 9999;
void connect_wifi();

UltraSonicDistanceSensor distanceSensor_1(D1, D2);
String s1name = "S1;";
UltraSonicDistanceSensor distanceSensor_2(D5, D6);
String s2name = "S2;";
WiFiClient client;

void setup() {
  Serial.begin(115200);
  connect_wifi();
}

void loop() {
  if (client.connect(host, port)) {
      client.print(s1name+distanceSensor_1.measureDistanceCm());
      client.print(s2name+distanceSensor_2.measureDistanceCm());
      delay(500);
  } else {
  Serial.println("connection failed, trying again in 1 second");
  delay(1000);}
}

void connect_wifi(){
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // set ESP to be wifi client
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  }
