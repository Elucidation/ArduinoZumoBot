#include <ros.h>
#include <ZumoMotors.h>
#include "Timer.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Distance between both wheels
#define WHEEL_OFFSET_M 0.1 // 10cm

#define LED_PIN 13
#define MAX_REAL_SPEED 0.65 // m/s
#define MAX_MOTOR_SPEED 400 // RPM

// Battery Voltage
#define BATTERY_READ_PIN 1

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


float getBatteryVoltage() {
  return 5.0 * analogRead(BATTERY_READ_PIN) / 1024.0;
}

void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  diff_drive(twist_msg.linear.x, twist_msg.angular.z);
}

// Command Velocity Subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);

// Battery Voltage Publisher
std_msgs::Float32 voltage_msg;
ros::Publisher battery_topic("battery_voltage", &voltage_msg);

void publishBatteryVoltage() {
  voltage_msg.data = getBatteryVoltage();
  battery_topic.publish( &voltage_msg );
}


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

  // Publish battery voltage every second
  t.every(1000, publishBatteryVoltage);
  
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