#include <ros.h>
#include <ZumoMotors.h>
#include "Timer.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Distance between both wheels
#define WHEEL_OFFSET_M 0.1 // 10cm

#define MAX_REAL_SPEED 0.65 // m/s
#define MAX_MOTOR_SPEED 400 // RPM

// Others
#define BATTERY_READ_PIN 1 // (SPECIAL) Analog Pin A1
#define USER_PUSHBUTTON_PIN 12
#define LED_PIN 13

// Encoders
#define ENCODER_LEFT_PIN_A 2
#define ENCODER_LEFT_PIN_B 11

#define ENCODER_RIGHT_PIN_A 4
#define ENCODER_RIGHT_PIN_B 2 // (SPECIAL) Analog pin A2
#define ENCODER_RIGHT_PIN_B_THRESHOLD 450 // Read value threshold crossing


// ROS
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;

ZumoMotors motors; // Motor controller
Timer t;


/// Left Encoder
int encoder_left_pos = 0;
int encoder_left_pin_A_last = LOW;
volatile int encoder_left_val = LOW;

// Right Encoder
int encode_right_pos = 0;
int encoder_right_pin_A_last = LOW;
volatile int encoder_right_val = LOW;

// Battery Voltage Publisher
std_msgs::Float32 voltage_msg;
ros::Publisher battery_topic("battery_voltage", &voltage_msg);

void publishBatteryVoltage() {
  voltage_msg.data = getBatteryVoltage();
  battery_topic.publish( &voltage_msg );
}


// Command Velocity Callback
void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  diff_drive(twist_msg.linear.x, twist_msg.angular.z);
}

// Command Velocity Subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);


////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // initialize serial:
  // Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  // inputString.reserve(200);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize Encoders
  pinMode (ENCODER_LEFT_PIN_A, INPUT);
  pinMode (ENCODER_LEFT_PIN_B, INPUT);
  pinMode (ENCODER_RIGHT_PIN_A, INPUT);
  // pinMode (encoder_right_pin_B, INPUT); // Analog
  
  // Only Right encoder works with interrupts (left pinB doesn't pass interrupt threshold of ~3.3V)
  attachInterrupt(0, measureRightEncoder, CHANGE); // 0 = pin 2 interrupt

  // Publish battery voltage every second
  t.every(1000, publishBatteryVoltage);
  
  // Left motor power wiring is reversed.
  motors.flipLeftMotor(true);
  
  // Initial speed is zero
  drive_stop();

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

////////////////////////////////////////////////////////////////////////////////

// cmd_x  : linear x velocity (forward velocity)
// cmd_th : angular z velocity (rotation of heading) 
void diff_drive(double cmd_x, double cmd_th) {
  int vl = (cmd_x - cmd_th * WHEEL_OFFSET_M / 2.0) * MAX_MOTOR_SPEED / MAX_REAL_SPEED;
  int vr = (cmd_x + cmd_th * WHEEL_OFFSET_M / 2.0) * MAX_MOTOR_SPEED / MAX_REAL_SPEED;
  motors.setLeftSpeed(vl);
  motors.setRightSpeed(vr);
}

// Drive left and right wheel motors for dt milliseconds, blocking
void drive(int vl, int vr) {
  motors.setLeftSpeed(vl);
  motors.setRightSpeed(vr);
}

// Stops motors
void drive_stop() {
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}

// Battery Voltage
float getBatteryVoltage() {
  return 5.0 * analogRead(BATTERY_READ_PIN) / 1024.0;
}

// Encoders
void measureLeftEncoder()
{
    encoder_left_val = digitalRead(ENCODER_RIGHT_PIN_A);
    if ((encoder_right_pin_A_last == LOW) && (encoder_left_val == HIGH))
    {
        if (analogRead(ENCODER_RIGHT_PIN_B) < ENCODER_RIGHT_PIN_B_THRESHOLD) 
          {encode_right_pos--;}
        else {encode_right_pos++;}
    }
    encoder_right_pin_A_last = encoder_left_val;
}

void measureRightEncoder()
{
    encoder_right_val = digitalRead(ENCODER_LEFT_PIN_A);
    if ((encoder_left_pin_A_last == LOW) && (encoder_right_val == HIGH))
    {
        if (digitalRead(ENCODER_LEFT_PIN_B) == LOW) {encoder_left_pos--;}
        else {encoder_left_pos++;}
    }
    encoder_left_pin_A_last = encoder_right_val;
}