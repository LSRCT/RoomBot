/*
  HCSR04 - Library for arduino, for HC-SR04 ultrasonic distance sensor.
  Created by Martin Sosic, June 11, 2016.
*/

#include "Arduino.h"
#include "USDS.h"

USDS::USDS(
  int triggerPin, int echoPin, int triggerPin2, int echoPin2) {
  this->triggerPin = triggerPin;
  this->echoPin = echoPin;
  this->triggerPin2 = triggerPin2;
  this->echoPin2 = echoPin2;
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void USDS::getDist() {
    //Using the approximate formula 19.307°C results in roughly 343m/s which is the commonly used value for air.
    return getDist(19.307);
}

void USDS::getDist(float temperature) {
  // Make sure that trigger pin is LOW.
  digitalWrite(triggerPin, LOW);
  digitalWrite(triggerPin2, LOW);
  delayMicroseconds(2);
  
  // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
  digitalWrite(triggerPin, HIGH);
  digitalWrite(triggerPin2, HIGH);
  delayMicroseconds(10);
  // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
  digitalWrite(triggerPin, LOW);
  double durationMicroSec = (double)(pulseIn(echoPin, HIGH));
  digitalWrite(triggerPin2, LOW);
  double durationMicroSec2 = (double)(pulseIn(echoPin2, HIGH)); 
  
  double speedOfSoundInCmPerMs = 0.03313 + 0.0000606 * temperature; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s
  
  double distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMs;
  double distanceCm2 = durationMicroSec2 / 2.0 * speedOfSoundInCmPerMs;
  this->dist = (long)(distanceCm2*10) << 16 | (long)(distanceCm*10);
  //this->dist[0] = (int)(distanceCm*10);
  //this->dist[1] = (int)(distanceCm2*10);
}
