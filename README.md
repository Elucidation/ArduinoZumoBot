ArduinoZumoBot
==============

Arduino code for controlling a custom-built pololu Zumo bot with encoder(s) 
and a pan/tilt servo setup.

Communicates over rosserial.


![Overview](http://i.imgur.com/VPFbfaw.jpg)


Wiring - Zumobot Arduino Uno
---
```

7  - Right Motor Direction
8  - Left Motor Direction
9  - Right Motor Speed (PWM)
10 - Left Motor Speed (PWM)

12 - User Pushbutton
13 - LED

3 - Left Encoder A (interrupt)
4 - Left Encoder B

2  - Right Encoder A (interrupt)
11 - Right Encoder B


A1 - Battery Voltage

A4/SDA - I2C 3-axis compass module
A5/SCL - I2C 3-axis compass module
```

Wiring - Arduino Pro Mini (Pan/Tilt and LED controller)
---
```
5  - Servo Pan (PWM)
6  - Servo Tilt (PWM)

3 - Flashlight LED left (PWM)
11 - Flashlight LED right (PWM)

A4/SDA - I2C Slave to RPi on address 0x04
A5/SCL - I2C Slave to RPi
```

Arduino Uno and Arduino Pro Mini
---

Interesting hardware limits were reached with just the one UNO, so a Pro Mini was added to control the servos and LEDs.

* The Uno only has 2Kb SRAM, which was rapidly reached with rosserial publishers, pan/tilt servos, battery voltage. 
* The clincher was the PID objects, which couldn't be used so a stripped-down functional version was used in place.
* With 3 Timers available, the original Servo.h library couldn't be used due to sharing the Timer with the ZumoMotors, so a Timer1-based Servo library had been used. But that interfered with the Buzzer on PWM pin 3. Combined with needing encoder interrupts, PWM motors and PWM servos, PWM lights etc. One board isn't enough.
* rosserial loses sync even at 115.2kbps with the arduino, probably due to the limited memory overflowing on the uno, removing some objects solved the problem, but it would cause transient halting errors occasionally, no good.