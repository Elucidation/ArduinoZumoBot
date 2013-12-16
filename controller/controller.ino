//#include <ZumoMotors.h>

#define LED_PIN 13

//ZumoMotors motors;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  pinMode(LED_PIN, OUTPUT);
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

void loop()
{
}

// Drive left and right wheel motors for dt milliseconds, blocking
void drive(int vl, int vr, int dt_millis) {
  motors.setLeftSpeed(vl);
  motors.setRightSpeed(vr);
  delay(dt_millis);
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
  int vel, vel2;
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
      // Both
      vel = Serial.parseInt();
      vel = constrain(vel, -400, 400);
      vel2 = Serial.parseInt();
      vel2 = constrain(vel2, -400, 400);
      Serial.print("Both ");
      Serial.print(vel);
      Serial.print(" ");
      Serial.println(vel2);
      drive(vel,vel2,250);
      break;
      
      case 'd':
      // Stopping
      Serial.println("Stop");
      drive(0,0,0);
      break;
    }
  }
}
