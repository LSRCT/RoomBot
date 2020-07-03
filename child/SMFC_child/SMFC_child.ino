#include <ESP8266WiFi.h>
#include <arduino.h>
//#include <HCSR04.h>

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

int ins = 0;

// Wifi variables
const char* ssid     = "LSRCT";
const char* password = "83067046472468411597";
const char* host = "192.168.178.27";
const uint16_t port = 9999;
void connect_wifi();
void exec_ins(int *);

int measureDistanceCm(const uint8_t *, const uint8_t *);

//UltraSonicDistanceSensor distanceSensor_1(D1, D2);
//String s1name = "S1;";
//UltraSonicDistanceSensor distanceSensor_2(D5, D6);
//String s2name = "S2;";
WiFiClient client;

void setup() {
  pinMode(D5, OUTPUT);
  pinMode(D6, INPUT);
  Serial.begin(115200);
  connect_wifi();
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  digitalWrite(D7, HIGH);
  digitalWrite(D8, HIGH);
}

void loop() {
  if (client.connect(host, port)) {
      //client.print(s1name+distanceSensor_1.measureDistanceCm());
      //client.print(distanceSensor_2.measureDistanceCm());
      client.println(measureDistanceCm(&D5,&D6));
      delay(10);
      while (client.available()) {
        ins = static_cast<int>(client.read());
        exec_ins(&ins);
      delay(90);
      //client.print(5);
      client.flush();}
  } else {
  Serial.println("connection failed, trying again in 1 second");
  delay(1000);}
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
  Serial.print(*ins);
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

int measureDistanceCm(const uint8_t *triggerPin, const uint8_t *echoPin) {
    // Make sure that trigger pin is LOW.
    digitalWrite(*triggerPin, LOW);
    delayMicroseconds(2);
    // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
    digitalWrite(*triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(*triggerPin, LOW);
    // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
    double durationMicroSec = (double)(pulseIn(*echoPin, HIGH));

    double speedOfSoundInCmPerMs = 0.03313 + 0.0000606 * 22.0; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s
    double distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMs;
    if (distanceCm == 0 || distanceCm > 400) {
        return -1.0 ;
    } else {
        return distanceCm*10.0;
    }
}
  
