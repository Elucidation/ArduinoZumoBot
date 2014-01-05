#include <ZumoMotors.h>
#include "Timer.h"

#define LED_PIN 13

ZumoMotors motors;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
Timer t;

void setup()
{
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Left motor power wiring is reversed.
  motors.flipLeftMotor(true);
  
  // Initial speed is zero
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

void loop()
{
  t.update();
}

// Drive left and right wheel motors for dt milliseconds, blocking
void drive(int vl, int vr, int dt_millis) {
  motors.setLeftSpeed(vl);
  motors.setRightSpeed(vr);
  t.after(dt_millis, drive_stop);
}

// Stops motors
void drive_stop() {
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  int vel, vel2, dt;
  while (Serial.available()) {
    // Get state character from buffer
    char c = (char)Serial.read();

    switch (c) {
      case 'a':
      // Drive
      vel = Serial.parseInt();
      vel = constrain(vel, -400, 400);
      Serial.print("Drive ");
      Serial.println(vel);
      drive(vel,vel,250);
      break;
      
      case 'b':
      // Turn
      vel = Serial.parseInt();
      vel = constrain(vel, -400, 400);
      Serial.print("Turn ");
      Serial.println(vel);
      drive(vel,-vel,250);
      break;
      
      case 'c':
      // c left_speed right_speed dt_millis
      // All
      vel = Serial.parseInt();
      vel = constrain(vel, -400, 400);
      vel2 = Serial.parseInt();
      vel2 = constrain(vel2, -400, 400);
      dt = Serial.parseInt();
      dt = constrain(dt, 0, 10000); // 0-10 seconds
      Serial.print("Both ");
      Serial.print(vel);
      Serial.print(" ");
      Serial.println(vel2);
      drive(vel,vel2,dt);
      break;
      
      case 'd':
      // Stopping
      Serial.println("Stop");
      drive_stop();
      break;
    }
  }
}
