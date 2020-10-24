# RoomBot
RoomBot is a cheap vacuum robot (25€) I upgraded for knowledge and _profit_.

## Robot
The robot was upgraded with:
- much bigger battery
- three ultrasonic distance sensors
- magnetometer
- ESP8266 for wireless controll

## Sensor data
The recorded distances as well as the magnetometer data is sent to SMFC_base.py running on a RaspberryPi via MQTTT. This programm logs the data for future evaluation.

## Localization
To localize the robot in my room a version of markov lovalization is used. Since the robot drives around randomly by design only the sensor data can be used to locate it.

Having only three distances and magnetometer data makes localization challanging, but also reveals how symmetry in the room influences the algorithm.
In the following picture the progressing of the probability map over six seconds is shown. Over that time, the intial uncertainty about the robots position in the room gets lower as the robot reaches a more unique part of the room.
![probability timeline](media/prob_timeline.png)

## Robot V2
Robot built from scratch, difference to v2 is the its controllable.  TODO
- three ultrasonic distance sensors
- 4 controllable motors
- ESP8266 for wireless controll
- ESP8266 for data transmission

## Localization 
Since V2 is controlled and does not drive around randomly, localization works much better. Markov localization is still used, with the difference being the added control input.
The gif below shows the robot driving through the room, being localized almost perfectly.
![probability gid](media/rb_loc_demo.gif)
