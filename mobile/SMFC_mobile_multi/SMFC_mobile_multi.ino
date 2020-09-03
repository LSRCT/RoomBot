#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <arduino.h>
//#include <HCSR04.h>
#include "USDS.h"

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

// robot control
void exec_ins(int *);
int ins = 0;

// Wifi variables
const char* ssid     = "LSRCT";
const char* password = "83067046472468411597";
const char* MQTT_BROKER = "192.168.178.27";
const char* host = "192.168.178.20";
const uint16_t port = 9999;
void connect_wifi();
void reconnectMQTT();
void callback(char*, byte*, unsigned int);
WiFiClient espClient;
PubSubClient MQTTclient(espClient);

// Sensor stuff
// trigger, echo, trigger2, echo2,...
USDS distSensor(D2, D1, D3, D0, D5, D6);



void setup() {
  Serial.begin(115200);
  connect_wifi();
  MQTTclient.setServer(MQTT_BROKER, 1883);
  MQTTclient.setCallback(callback);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  digitalWrite(D7, HIGH);
  digitalWrite(D8, HIGH);
}

void reconnectMQTT() {
    while (!MQTTclient.connected()) {
        Serial.println("Reconnecting MQTT...");
        if (!MQTTclient.connect("ESP8266Client_drive")) {
            Serial.print("failed, rc=");
            Serial.print(MQTTclient.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        } else {
          MQTTclient.subscribe("RR/driveIns");
          }
        }
    Serial.println("MQTT Connected...");
}

void loop() {
  if (!MQTTclient.connected()) {
        reconnectMQTT();
  }
  MQTTclient.loop();
  distSensor.getDist();
  Serial.println(distSensor.dist);
  MQTTclient.publish("RR/sensors", distSensor.dist, 6);
  delay(90);
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived");
  Serial.println(String(topic));
  if (String(topic) == "RR/driveIns"){
    ins = payload[0];
    //Serial.println(ins);
    exec_ins(&ins);
  }
}


void exec_ins(int *ins){
  if (*ins == 48){
    digitalWrite(D7, HIGH);
    digitalWrite(D8, HIGH);
    }
    if (*ins == 49){
    digitalWrite(D7, LOW);
    digitalWrite(D8, HIGH);
    }
   if (*ins == 50){
    digitalWrite(D7, HIGH);
    digitalWrite(D8, LOW);
    }
   Serial.println(*ins);
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

  
