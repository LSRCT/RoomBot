#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <arduino.h>
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

#define DRIVE_PIN D8


// robot control
void exec_ins(int *);
int ins = 0;

// Wifi variables
const char* ssid     = "LSRCT";
const char* password = "83067046472468411597";
const char* MQTT_BROKER = "192.168.178.27";


// setup functions
void setupWifi();
void setupMotor();
void setupMQTT();

// MQTT stuff
void reconnectMQTT();
void callback(char*, byte*, unsigned int);
WiFiClient espClient;
PubSubClient MQTTclient(espClient);

// Sensor stuff
// trigger, echo, trigger2, echo2,...
USDS distSensor(D2, D1);

void setup() {
  Serial.begin(115200);
  setupWifi();
  setupMQTT();
  setupMotor();
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

void setupMQTT(){
  MQTTclient.setServer(MQTT_BROKER, 1883);
  MQTTclient.setCallback(callback);
}

void setupMotor(){
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, HIGH);
  }

void setupWifi(){
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
  if (!MQTTclient.connected()) {
        reconnectMQTT();
  }
  MQTTclient.loop();
  distSensor.getDist();
  MQTTclient.publish("RR/sensors", distSensor.dist, 6);
  delay(90);
}



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived");
  Serial.println(String(topic));
  if (String(topic) == "RR/driveIns"){
    ins = payload[0];
    exec_ins(&ins);
  }
}


void exec_ins(int *ins){
  if (*ins == 48){
    digitalWrite(DRIVE_PIN, HIGH);
    }
    if (*ins == 49){
    digitalWrite(DRIVE_PIN, LOW);
    }
   if (*ins == 50){
    digitalWrite(DRIVE_PIN, HIGH);
    }
  }


  
