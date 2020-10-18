#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <arduino.h>
#include "USDS.h"
#include <MPU9250_asukiaaa.h>


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

#define SDA_PIN D4
#define SCL_PIN D5

//imu
MPU9250_asukiaaa IMU;
float mX, mY, mZ;
byte magXY[4] = {0};


// Wifi variables
const char* ssid     = "LSRCT";
const char* password = "83067046472468411597";
const char* MQTT_BROKER = "192.168.178.27";


// setup functions
void setupWifi();
void setupIMU();
void setupMQTT();
void publish_mag();

// MQTT stuff
void reconnectMQTT();
WiFiClient espClient;
PubSubClient MQTTclient(espClient);

// Sensor stuff
// trigger, echo, trigger2, echo2,...
USDS distSensor(D2, D1, D3, D0, D7, D6);

void setup() {
  Serial.begin(115200);
  setupIMU();
  setupWifi();
  setupMQTT();
}

void reconnectMQTT() {
  /*
   * Connect to the MQTT broker
   */
    while (!MQTTclient.connected()) {
        Serial.println("Reconnecting MQTT...");
        if (!MQTTclient.connect("ESP8266Client_sense")) {
            Serial.print("failed, rc=");
            Serial.print(MQTTclient.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        } 
        }
    Serial.println("MQTT Connected...");
}

void setupMQTT(){
  MQTTclient.setServer(MQTT_BROKER, 1883);
}

void setupIMU(){
  /*
   * Setup the IMU pins, wire connection and magnetometer sampling rate
   */
  Wire.begin(SDA_PIN, SCL_PIN);
  IMU.setWire(&Wire);
  IMU.beginMag(MAG_MODE_CONTINUOUS_100HZ);
}

void setupWifi(){
  /*
   * Connect to the wifi network
   */
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
  publish_mag();
  //delay(80);
}

void publish_mag(){
  /*
   *Publish magnetometer X and Y to "RR/magnetometer"
   *Encoded as 4 bytes, offset so its not negative
   *Averages over 100 ms -> this function takes 100 ms
  */
  mX = 0;
  mY = 0;
  // avg over 100 ms
  for(int i=0; i<10;i++){
    IMU.magUpdate();
    //mX += IMU.magHorizDirection()+180;
    mX += IMU.magX();
    mY += IMU.magY();
    delay(10);
    }
  // make sure its not negativem 200 is arbitrary
  mY = (mY)+2000;
  mX = (mX)+2000;
  magXY[0] = int(mX)>> 8;
  magXY[1] = int(mX);
  magXY[2] = int(mY)>> 8;
  magXY[3] = int(mY);
  MQTTclient.publish("RR/magnetometer", magXY, 4);
  }


  
