/*
  HCSR04 - Library for arduino, for HC-SR04 ultrasonic distance sensor.
  Created by Martin Sosic, June 11, 2016.
  Modified by AN
*/

#include "Arduino.h"
#include "USDS.h"

USDS::USDS(int triggerPin, int echoPin) {
  this->triggerPin = triggerPin;
  this->echoPin = echoPin;
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void USDS::getDist() {
    //Using the approximate formula 19.307°C results in roughly 343m/s which is the commonly used value for air.
    return getDist(19.307);
}

void USDS::getDist(float temperature) {
  // Make sure that trigger pin is LOW.
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
   
  // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
  digitalWrite(triggerPin, LOW);
  double durationMicroSec = (double)(pulseIn(echoPin, HIGH));
  double speedOfSoundInCmPerMs = 0.03313 + 0.0000606 * temperature; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s
  double distanceCm = ((durationMicroSec / 2.0 * speedOfSoundInCmPerMs)*10.0);
  this->dist[0] = (int(distanceCm)>> 8)& 0xFF;
  this->dist[1] = (int(distanceCm))& 0xFF;
  this->dist[2] = (int(distanceCm) >> 8)& 0xFF;
  this->dist[3] = (int(distanceCm))& 0xFF;
  this->dist[4] = (int(distanceCm) >> 8)& 0xFF;
  this->dist[5] = (int(distanceCm))& 0xFF;

}
