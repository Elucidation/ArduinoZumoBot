#include <ros.h>
#include <ZumoMotors.h>
#include "Timer.h"
#include <geometry_msgs/Twist.h>

// Distance between both wheels
#define WHEEL_OFFSET_M 0.1 // 10cm

#define LED_PIN 13
#define MAX_REAL_SPEED 0.65 // m/s
#define MAX_MOTOR_SPEED 400 // RPM

ZumoMotors motors;

// String inputString = "";         // a string to hold incoming data
// boolean stringComplete = false;  // whether the string is complete
Timer t;

/// Encoder
int encoder0PinA = 2;
int encoder0PinB = 11;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
volatile int n = LOW;
int last = 0;

int encoder1PinA = 5;
int encoder1PinB = A2;
int encoder1Pos = 0;
int encoder1PinALast = LOW;
volatile int n2 = LOW;
int last1 = 0;


ros::NodeHandle  nh;
geometry_msgs::Twist twist_msg;


void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  diff_drive(twist_msg.linear.x, twist_msg.angular.z);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);



void measureLeftEncoder()
{
    n = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (n == HIGH))
    {
        if (analogRead(encoder1PinB) < 450)
        {
            encoder1Pos--;
        }
        else
        {
            encoder1Pos++;
        }
    }
    encoder1PinALast = n;
}

void measureRightEncoder()
{
    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH))
    {
        if (digitalRead(encoder0PinB) == LOW)
        {
            //if (analogRead(encoder0PinB) < 450) {
            encoder0Pos--;
        }
        else
        {
            encoder0Pos++;
        }
    }
    encoder0PinALast = n;
}
void printStuff()
{
    if (last != encoder0Pos)
    {
        // Serial.print ("A");
        // Serial.println (encoder0Pos);
        last = encoder0Pos;
    }

    if (last1 != encoder1Pos)
    {
        // Serial.print ("B");
        // Serial.println (encoder1Pos);
        last1 = encoder1Pos;
    }
}
///

void setup()
{
  // initialize serial:
  // Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  // inputString.reserve(200);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize Encoders
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  pinMode (encoder1PinA, INPUT);
  // pinMode (encoder1PinB, INPUT); // Analog
  attachInterrupt(0, measureRightEncoder, CHANGE);
  
  // Print encoder values every second
  // t.every(1000, printStuff);
  
  // Left motor power wiring is reversed.
  motors.flipLeftMotor(true);
  
  // Initial speed is zero
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  nh.initNode();
  nh.subscribe(cmd_vel_topic);
}

void loop()
{
  // Measure left encoder manually
  measureLeftEncoder();
  
  // Timer updates
  t.update();

  nh.spinOnce();
}


// cmd_x  : linear x velocity (forward velocity)
// cmd_th : angular z velocity (rotation of heading) 
void diff_drive(double cmd_x, double cmd_th) {
  int vl = (cmd_x - cmd_th * WHEEL_OFFSET_M / 2.0) * MAX_MOTOR_SPEED / MAX_REAL_SPEED;
  int vr = (cmd_x + cmd_th * WHEEL_OFFSET_M / 2.0) * MAX_MOTOR_SPEED / MAX_REAL_SPEED;
  motors.setLeftSpeed(vl);
  motors.setRightSpeed(vr);
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
// void serialEvent() {
//   int vel, vel2, dt;
//   while (Serial.available()) {
//     // Get state character from buffer
//     char c = (char)Serial.read();

//     switch (c) {
//       case 'a':
//       // Drive
//       vel = Serial.parseInt();
//       vel = constrain(vel, -400, 400);
//       Serial.print("Drive ");
//       Serial.println(vel);
//       drive(vel,vel,250);
//       break;
      
//       case 'b':
//       // Turn
//       vel = Serial.parseInt();
//       vel = constrain(vel, -400, 400);
//       Serial.print("Turn ");
//       Serial.println(vel);
//       drive(vel,-vel,250);
//       break;
      
//       case 'c':
//       // c left_speed right_speed dt_millis
//       // All
//       vel = Serial.parseInt();
//       vel = constrain(vel, -400, 400);
//       vel2 = Serial.parseInt();
//       vel2 = constrain(vel2, -400, 400);
//       dt = Serial.parseInt();
//       dt = constrain(dt, 0, 10000); // 0-10 seconds
//       Serial.print("Both ");
//       Serial.print(vel);
//       Serial.print(" ");
//       Serial.println(vel2);
//       drive(vel,vel2,dt);
//       break;
      
//       case 'd':
//       // Stopping
//       Serial.println("Stop");
//       drive_stop();
//       break;
//     }
//   }
// }
