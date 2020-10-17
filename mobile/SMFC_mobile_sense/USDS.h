/*
  HCSR04 - Library for arduino, for HC-SR04 ultrasonic distance sensor.
  Created by Martin Sosic, June 11, 2016.
  Modified by AN
*/

#ifndef USDS_H
#define USDS_H

#include "Arduino.h"

class USDS {
 public:
    /**
     * @param triggerPin  Digital pin that is used for controlling sensor (output).
     * @param echoPin  Digital pin that is used to get information from sensor (input).
     */
    USDS(int triggerPin, int echoPin, int triggerPin2, int echoPin2, int triggerPin3, int echoPin3);

    /**
     * Measures distance by sending ultrasonic waves and measuring time it takes them
     * to return.
     * @returns Distance in centimeters, or negative value if distance is greater than 400cm.
     */
    void getDist();
    byte dist[6] = {0};
    /**
     * Measures distance by sending ultrasonic waves and measuring time it takes them
     * to return.
     * @param temperature  Temperature in degrees celsius
     * @returns Distance in centimeters, or negative value if distance is greater than 400cm.
     */
    void getDist(float temperature);
 private:
    int triggerPin, echoPin, triggerPin2, echoPin2, triggerPin3, echoPin3;
};

#endif
