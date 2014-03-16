ArduinoZumoBot
==============

Arduino code for controlling a custom-built pololu Zumo bot with encoder(s) 
and a pan/tilt servo setup.

Communicates over rosserial.


Wiring
---

In Order:

```
2  - Right Encoder A
3  - Buzzer
4  - Left Encoder A
5  - Servo Pan
6  - Servo Tilt
7  - Right Motor Direction
8  - Left Motor Direction
9  - Right Motor Speed
10 - Left Motor Speed
11 - Right Encoder B
12 - User Pushbutton
13 - LED

A0 -
A1 - Battery Voltage
A2 - Left Encoder B
A3 -
A4/SDA - I2C 3-axis compass module
A5/SCL - I2C 3-axis compass module
```

In parts:

```
5  - Servo Pan
6  - Servo Tilt

7  - Right Motor Direction
8  - Left Motor Direction
9  - Right Motor Speed
10 - Left Motor Speed

12 - User Pushbutton
13 - LED

2  - Right Encoder A
11 - Right Encoder B

4  - Left Encoder A
A2 - Left Encoder B

A1 - Battery Voltage

A4/SDA - I2C 3-axis compass module
A5/SCL - I2C 3-axis compass module
```