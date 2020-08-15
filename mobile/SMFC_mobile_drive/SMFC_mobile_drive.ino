
#include <ESP8266WiFi.h>
#include <arduino.h>
#include <PubSubClient.h>

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
int insL = 0;
int insR = 0;

// PINS
int motorpin_L_fwd = D5;
int motorpin_L_bwd = D6;
int motorpin_R_fwd = D7;
int motorpin_R_bwd = D8;
 
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


void setup() {
  Serial.begin(115200);
  connect_wifi();
  MQTTclient.setServer(MQTT_BROKER, 1883);
  MQTTclient.setCallback(callback);
  analogWrite(motorpin_L_fwd, 0);
  analogWrite(motorpin_L_bwd, 0);
  analogWrite(motorpin_R_fwd, 0);
  analogWrite(motorpin_R_bwd, 0);
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
}

void exec_ins(int *insL, int *insR){
  if (*insL >= 0){
    analogWrite(motorpin_L_bwd, 0);
    analogWrite(motorpin_L_fwd, *insL);
    } else {
    analogWrite(motorpin_L_fwd, 0);
    analogWrite(motorpin_L_bwd, abs(*insL));
  }
  if (*insR >= 0){
    analogWrite(motorpin_R_fwd, *insR);
    analogWrite(motorpin_R_bwd, 0);
  } else {
    analogWrite(motorpin_R_bwd, abs(*insR));
    analogWrite(motorpin_R_fwd, 0);
  } 
  Serial.print(*insL);
  Serial.println(*insR);
  }

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived");
  Serial.println(String(topic));
  if (String(topic) == "RR/driveIns"){
    insL = payload[0]+ (payload[1] << 8) - 1024;
    insR = payload[2]+ (payload[3] << 8) - 1024;
    Serial.println(insL);
    Serial.println(insR);
    exec_ins(&insL, &insR);
  }
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

  
