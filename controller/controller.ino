#include <ros.h> // Timer.h seems to bugger it up, so nope.
#include <rosserial_arduino/pan_tilt.h> // {pan:uint8,tilt:uint8} ros msg

#include <ZumoMotors.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Nice Servo Library that uses Timer 2 (since Timer1 used by ZumoMotors.h)
// Unfortunately Buzzer also uses Timer 2 so can't be used at the same time.
// Library : https://github.com/DerekK19/Arduino/tree/master/libraries/ServoTimer2
#include <ServoTimer2.h> 

// Distance between both wheels
#define WHEEL_OFFSET_M 0.1 // 10cm

#define MAX_REAL_SPEED 0.65 // m/s
#define MAX_MOTOR_SPEED 400 // RPM

// Others
#define BATTERY_READ_PIN 1 // (SPECIAL) Analog Pin A1
#define USER_PUSHBUTTON_PIN 12
#define LED_PIN 13

#define BATTERY_VOLTAGE_PUBLISH_RATE 1000 // ms (publish voltage every second)

// Encoders
#define ENCODER_LEFT_PIN_A 2
#define ENCODER_LEFT_PIN_B 11

#define ENCODER_RIGHT_PIN_A 4
#define ENCODER_RIGHT_PIN_B 2 // (SPECIAL) Analog pin A2
#define ENCODER_RIGHT_PIN_B_THRESHOLD 450 // Read value threshold crossing

// Pan Tilt Servo pins
#define SERVO_PAN_PIN 5 // Pan
#define SERVO_TILT_PIN 6 // Tilt

// Pan servo center min/max range (0-180 degrees with 90 degrees centred)
#define SERVO_PAN_MIN 40
#define SERVO_PAN_MAX 140

// Tilt servo center min/max range (0-180 degrees with 90 degrees centred)
#define SERVO_TILT_MIN 50
#define SERVO_TILT_MAX 130
// ServoTimer2 min/max microsecond delays 544us - 2400us ~ 0 - 180 degrees
#define ST2_MIN 544
#define ST2_MAX 2400

#define PAN_TILT_MOVE_DELAY 2 // ms delay for smooth movement

// ROS
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;

ZumoMotors motors; // Motor controller
// Timer t;

// Pan Tilt variables
uint8_t pan, tilt;

void set_position_callback(const rosserial_arduino::pan_tilt& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  pan = msg.pan;
  tilt = msg.tilt;
  
  moveTo(int(pan), int(tilt));
}

// Set up pan/tilt subscriber
ros::Subscriber<rosserial_arduino::pan_tilt> pan_tilt_sub("pantilt/set_position", set_position_callback);

// Servo objects
ServoTimer2 servo_pan;
ServoTimer2 servo_tilt;

// Servo positions (0-180 degrees with 90 degrees centred)
int pan_pos = 0;
int tilt_pos = 0;

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
ros::Publisher battery_pub("battery_voltage", &voltage_msg);

void publishBatteryVoltage() {
  voltage_msg.data = getBatteryVoltage();
  battery_pub.publish( &voltage_msg );
}


// Command Velocity Callback
void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  diff_drive(twist_msg.linear.x, twist_msg.angular.z);
}

// Command Velocity Subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);

unsigned long last_battery_published = 0;
////////////////////////////////////////////////////////////////////////////////

void setup()
{  
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize Encoders
  pinMode (ENCODER_LEFT_PIN_A, INPUT);
  pinMode (ENCODER_LEFT_PIN_B, INPUT);
  pinMode (ENCODER_RIGHT_PIN_A, INPUT);
  // pinMode (encoder_right_pin_B, INPUT); // Analog
  
  // Only Right encoder works with interrupts (left pinB doesn't pass interrupt threshold of ~3.3V)
  attachInterrupt(0, measureRightEncoder, CHANGE); // 0 = pin 2 interrupt
  
  // Left motor power wiring is reversed.
  motors.flipLeftMotor(true);
  
  // Initial speed is zero
  drive_stop();

  // Attach Pan Tilt Servos
  servo_pan.attach(SERVO_PAN_PIN);
  servo_tilt.attach(SERVO_TILT_PIN);

  // Init pan/tilt to center
  setZero();

  nh.initNode();
  nh.advertise(battery_pub); // Battery Voltage Publisher

  nh.subscribe(cmd_vel_topic); // Command Velocity Subscriber
  nh.subscribe(pan_tilt_sub); // Pan/Tilt Subscriber

  
  last_battery_published = millis();
}

void loop()
{
  // Measure left encoder manually
  measureLeftEncoder();
  
  // Publish battery voltage every second
  if (millis() - last_battery_published > BATTERY_VOLTAGE_PUBLISH_RATE)
  {
    publishBatteryVoltage();
    last_battery_published = millis();
  }

  nh.spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
// Motor drive speeds
int vl = 0, vr = 0;

// cmd_x  : linear x velocity (forward velocity)
// cmd_th : angular z velocity (rotation of heading) 
void diff_drive(double cmd_x, double cmd_th) {
  vl = (cmd_x - cmd_th * WHEEL_OFFSET_M / 2.0) * MAX_MOTOR_SPEED / MAX_REAL_SPEED;
  vr = (cmd_x + cmd_th * WHEEL_OFFSET_M / 2.0) * MAX_MOTOR_SPEED / MAX_REAL_SPEED;
  drive();
}

// Drive left and right wheel motors vased on vl and vr
void drive() {
  motors.setLeftSpeed(vl);
  motors.setRightSpeed(vr);
}

// Stops motors
void drive_stop() {
  vl = 0;
  vr = 0;
  drive();
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


// Smoothly move to pan tilt pos 0-100 range for both (50 centered)
void moveTo(int pan, int tilt)
{
  if (vl != 0 || vr != 0) {
    // WARNING
    // Left Encoder will miss steps due to the delay in this while loop!
  }
  while (abs(pan_pos - pan) + abs(tilt_pos - tilt) > 0) {
    pan_pos += pan_pos > pan ? -1 : pan_pos < pan ? 1 : 0;
    tilt_pos += tilt_pos > tilt ? -1 : tilt_pos < tilt ? 1 : 0;
    setPan( map(pan, 0, 100, SERVO_PAN_MIN, SERVO_PAN_MAX) );
    setTilt( map(tilt, 0, 100, SERVO_TILT_MIN, SERVO_TILT_MAX) );
    delay(PAN_TILT_MOVE_DELAY); // ms
  }
}


// Moves to pan and tilt in range 0 to 100, where 50 is centered
void setPanTilt(int pan, int tilt)
{
  setPan( map(pan, 0, 100, SERVO_PAN_MIN, SERVO_PAN_MAX) );
  setTilt( map(tilt, 0, 100, SERVO_TILT_MIN, SERVO_TILT_MAX) );
}

// Center Pan/Tilt servos
void setZero()
{
  moveTo(50,50);
}

// Given degree, constrain to 0-180, then convert to microsecond delay
void setPan(int pan_deg)
{
  pan_deg = constrain(pan_deg, SERVO_PAN_MIN, SERVO_PAN_MAX);
  servo_pan.write(map(pan_deg, 0, 180, ST2_MIN, ST2_MAX));
}

void setTilt(int tilt_deg)
{
  tilt_deg = constrain(tilt_deg, SERVO_TILT_MIN, SERVO_TILT_MAX);
  servo_tilt.write(map(tilt_deg, 0, 180, ST2_MIN, ST2_MAX));
}