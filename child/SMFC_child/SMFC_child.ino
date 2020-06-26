#include <ESP8266WiFi.h>

#ifndef STASSID
#define STASSID "LSRCT"
#define STAPSK  "83067046472468411597"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "192.168.178.20";
const uint16_t port = 9999;

void connect_wifi();
WiFiClient client;

void setup() {
  Serial.begin(115200);
  connect_wifi();
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

void loop() {
  if (!client.connect(host, port)) {
    Serial.println("connection failed, trying again in 1 second");
    delay(1000);
  }
  client.println("LOCDATA");
  client.println("DATA1");
  delay(1000);
}
