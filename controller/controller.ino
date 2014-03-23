#include <ros.h> // Timer.h seems to bugger it up, so nope.
#include <rosserial_arduino/pan_tilt.h> // {pan:uint8,tilt:uint8} ros msg

// ZumoMotors uses Timer1
// So Servo has to use Timer 2
// Since Timer 2 is used for Servo,  analogWrite of PWM on pins 3 and 11 is disabled

#include <ZumoMotors.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Nice Servo Library that uses Timer 2 (since Timer1 used by ZumoMotors.h)
// Unfortunately Buzzer also uses Timer 2 so can't be used at the same time.
// Library : https://github.com/DerekK19/Arduino/tree/master/libraries/ServoTimer2
#include <ServoTimer2.h> 

// PID controllers for wheels
// #include <PID_v1.h> // Couldn't use, ran out of memory, using function-based PID

#define ROS_SERIAL_BAUD_RATE 115200 // Rosserial baud rate 115200 bps for example

// Distance between both wheels
#define WHEEL_OFFSET_M 0.0889 // 3.5" ~ 8.89cm
#define HALF_WHEEL_OFFSET_M 0.04445 // distance from center to wheel along turning axis

#define MAX_REAL_SPEED 0.65 // m/s
#define MAX_MOTOR_SPEED 400.0 // RPM

// Others
#define BATTERY_READ_PIN 1 // (SPECIAL) Analog Pin A1
#define USER_PUSHBUTTON_PIN 12
#define LED_PIN 13

#define BATTERY_VOLTAGE_PUBLISH_RATE 1000 // ms (publish voltage every second)

// Encoders
#define ENCODER_LEFT_PIN_A 3
#define ENCODER_LEFT_PIN_B 4

#define ENCODER_RIGHT_PIN_A 2
#define ENCODER_RIGHT_PIN_B 11


#define TICKS_PER_REV 379 // ticks for one wheel revolution
#define TICKS_PER_METER 3110 // 790 ticks for 10 inches, 790 ticks per .254m ~ 3110.23622 ticks/meter
#define METERS_PER_REV 0.12186495176848874 // meters per full revolution
#define REV_PER_METER 8.20580474934037 // revolutions per meter


#define WHEEL_VEL_PUBLISH_RATE 100 // ms (10Hz)
#define MOTOR_PID_RATE 20 // ms (50Hz) PID Motor update rate (Should be < WHEEL_VEL_PUBLISH_RATE)
#define MOTOR_SPEED_CUTOFF 20 // lowest motor speed at which to zero it

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


#define ENABLE_PAN_TILT true

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

// Left Encoder
long encoder_left_ticks = 0;
long encoder_left_ticks_old = 0;
int encoder_left_pin_A_last = LOW;
int encoder_left_val = LOW;
float encoder_left_velocity = 0;

// Right Encoder
long encoder_right_ticks = 0;
long encoder_right_ticks_old = 0;
int encoder_right_pin_A_last = LOW;
int encoder_right_val = LOW;
float encoder_right_velocity = 0;

unsigned long encoder_last_velocity_time = 0; // microseconds
long last_wheel_ticks_published = 0;

// Motor PIDs
double pid_left_input = 0, pid_left_output = 0, pid_left_setpoint = 0;
double pid_right_input = 0, pid_right_output = 0, pid_right_setpoint = 0;

// PID pid_left(&pid_left_input, &pid_left_output, &pid_left_setpoint, 20, 500, 0, DIRECT);
// PID pid_right(&pid_right_input, &pid_right_output, &pid_right_setpoint, 20, 500, 0, DIRECT);
unsigned long last_motor_pid_updated = 0;


// Encoder tick publishers
std_msgs::Float32 wheel_vel;
ros::Publisher lwheel_pub("wheel_velocity/left", &wheel_vel);
ros::Publisher rwheel_pub("wheel_velocity/right", &wheel_vel);

// Publish left and right wheel encoder ticks
void publishWheelVelocity() {
  // updateWheelPositionVelocity();

  // encoder velocities are in rev/sec, convert to m/s
  wheel_vel.data = encoder_left_velocity * METERS_PER_REV;
  lwheel_pub.publish(&wheel_vel);

  wheel_vel.data = encoder_right_velocity * METERS_PER_REV;
  rwheel_pub.publish(&wheel_vel);
}

// Battery Voltage Publisher
std_msgs::Float32 voltage_msg;
ros::Publisher battery_pub("battery_voltage", &voltage_msg);

void publishBatteryVoltage() {
  voltage_msg.data = getBatteryVoltage();
  battery_pub.publish( &voltage_msg );
}


// Command Velocity Callback
void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  // messages in meters per second
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
  pinMode (ENCODER_RIGHT_PIN_B, INPUT);
  
  // Both Servos and Encoders use Timer1 library
  if (ENABLE_PAN_TILT)
  {
    // Attach Pan Tilt Servos
    servo_pan.attach(SERVO_PAN_PIN);
    servo_tilt.attach(SERVO_TILT_PIN);
  } 

  // Attach Encoder Interrupts
  attachInterrupt(1, measureLeftEncoder, CHANGE); // 1 = pin 3 interrupt
  attachInterrupt(0, measureRightEncoder, CHANGE); // 0 = pin 2 interrupt

  // Left motor power wiring is reversed.
  motors.flipLeftMotor(true);
  
  // Initial speed is zero
  drive(0, 0);

  // Initialize PIDs
  initPIDs();

  // Init pan/tilt to center
  setPanTiltZero();

  nh.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  nh.initNode();
  nh.advertise(battery_pub); // Battery Voltage Publisher

  // Encoder publishers
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);

  nh.subscribe(cmd_vel_topic); // Command Velocity Subscriber
  if (ENABLE_PAN_TILT) {
    nh.subscribe(pan_tilt_sub); // Pan/Tilt Subscriber
  }

  last_battery_published = millis();
}

void loop()
{  
  // Publish battery voltage every second
  if (millis() - last_battery_published > BATTERY_VOLTAGE_PUBLISH_RATE)
  {
    publishBatteryVoltage();
    last_battery_published = millis();
  }

  // Publish encoder at 100 Hz
  if (millis() - last_wheel_ticks_published > WHEEL_VEL_PUBLISH_RATE)
  {
    publishWheelVelocity();
    last_wheel_ticks_published = millis();
  }

  if (millis() - last_motor_pid_updated > MOTOR_PID_RATE)
  {
    updateMotorPIDs();
    last_motor_pid_updated = millis();
  }

  nh.spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
// Motor drive

// cmd_x  : linear x velocity (forward velocity) m/s
// cmd_th : angular z velocity (rotation of heading) rad/s
void diff_drive(double cmd_x, double cmd_th) {
  // cmd_x in m/s
  double vl = cmd_x - cmd_th * HALF_WHEEL_OFFSET_M;
  double vr = cmd_x + cmd_th * HALF_WHEEL_OFFSET_M;

  // velocities in m/s, convert to rev/s
  pid_left_setpoint = vl * REV_PER_METER;
  pid_right_setpoint = vr * REV_PER_METER;
  // values used by PID loop
}

void drive(int lspeed, int rspeed)
{
  motors.setLeftSpeed(lspeed);
  motors.setRightSpeed(rspeed);
}

// Battery Voltage
float getBatteryVoltage() {
  return 5.0 * analogRead(BATTERY_READ_PIN) / 1024.0;
}

// Encoders
void measureLeftEncoder()
{
    encoder_left_val = digitalRead(ENCODER_LEFT_PIN_A);
    if ((encoder_left_pin_A_last == LOW) && (encoder_left_val == HIGH))
    {
        if (digitalRead(ENCODER_LEFT_PIN_B) == LOW) {encoder_left_ticks--;}
        else {encoder_left_ticks++;}
    }
    encoder_left_pin_A_last = encoder_left_val;
}

void measureRightEncoder()
{
    encoder_right_val = digitalRead(ENCODER_RIGHT_PIN_A);
    if ((encoder_right_pin_A_last == LOW) && (encoder_right_val == HIGH))
    {
        if (digitalRead(ENCODER_RIGHT_PIN_B) == LOW) {encoder_right_ticks--;} // Transistor Flipped? 
        else {encoder_right_ticks++;}
    }
    encoder_right_pin_A_last = encoder_right_val;
}


void updateWheelPositionVelocity()
{
  long dt = micros() - encoder_last_velocity_time; // ms

  // ticks offset
  long dleft = encoder_left_ticks - encoder_left_ticks_old;
  long dright = encoder_right_ticks - encoder_right_ticks_old;

  // Velocity in rev/second
  // 1e6 because dt is in microseconds
  encoder_left_velocity = (float)(1000000*dleft) / (TICKS_PER_REV * dt);
  encoder_right_velocity = (float)(1000000*dright) / (TICKS_PER_REV * dt);

  // Update PID inputs
  pid_left_input = (double)encoder_left_velocity;
  pid_right_input = (double)encoder_right_velocity;

  // Update previous values
  encoder_left_ticks_old = encoder_left_ticks;
  encoder_right_ticks_old = encoder_right_ticks;
  encoder_last_velocity_time = micros();
}

void updateMotorPIDs()
{
  // Update wheel velocities (pid inputs)
  updateWheelPositionVelocity();

  // run pid controller, setting pid outputs
  // pid_left.Compute();
  // pid_right.Compute();
  doLeftPID();
  doRightPID();

  // If desired motor speed is less than threshold, zero it out
  if (abs(pid_left_output) < MOTOR_SPEED_CUTOFF) { pid_left_output = 0; }
  if (abs(pid_right_output) < MOTOR_SPEED_CUTOFF) { pid_right_output = 0; }

  // Set motor speeds
  motors.setLeftSpeed(pid_left_output);
  motors.setRightSpeed(pid_right_output);
}

// Init PIDs for zumo motors
void initPIDs()
{
  // Init setpoints in case they aren't yet set
  pid_left_setpoint = 0;
  pid_right_setpoint = 0;

  // pid_left.SetMode(AUTOMATIC);
  // pid_left.SetOutputLimits(-400,400);
  // pid_left.SetSampleTime(20);
  // pid_right.SetMode(AUTOMATIC);
  // pid_right.SetOutputLimits(-400,400);
  // pid_right.SetSampleTime(20);
}

// Smoothly move to pan tilt pos 0-100 range for both (50 centered)
void moveTo(int pan, int tilt)
{
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
void setPanTiltZero()
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


#define PID_OUTMIN -400
#define PID_OUTMAX 400

#define KP 20
#define KI 10 // 500 * 20ms = 10
#define KD 0

// Had to do PIDs unclassed manually since we're running out of memory.
double left_ITerm = 0;
double left_lastInput = 0;
void doLeftPID()
{
    /* Compute all the working error variables */
    double error = pid_left_setpoint - pid_left_input;
    left_ITerm += (KI * error);
    if(left_ITerm > PID_OUTMAX) left_ITerm= PID_OUTMAX;
    else if(left_ITerm < PID_OUTMIN) left_ITerm= PID_OUTMIN;
    double dInput = (pid_left_input - left_lastInput);

    /* Compute PID Output */
    pid_left_output = KP * error + left_ITerm- KD * dInput;
      
    if(pid_left_output > PID_OUTMAX) pid_left_output = PID_OUTMAX;
    else if(pid_left_output < PID_OUTMIN) pid_left_output = PID_OUTMIN;
  
    /* Remember some variables for next time */
    left_lastInput = pid_left_input;
}

double right_ITerm = 0;
double right_lastInput = 0;
void doRightPID()
{
    /* Compute all the working error variables */
    double error = pid_right_setpoint - pid_right_input;
    right_ITerm += (KI * error);
    if(right_ITerm > PID_OUTMAX) right_ITerm= PID_OUTMAX;
    else if(right_ITerm < PID_OUTMIN) right_ITerm= PID_OUTMIN;
    double dInput = (pid_right_input - right_lastInput);

    /* Compute PID Output */
    pid_right_output = KP * error + right_ITerm- KD * dInput;
      
    if(pid_right_output > PID_OUTMAX) pid_right_output = PID_OUTMAX;
    else if(pid_right_output < PID_OUTMIN) pid_right_output = PID_OUTMIN;
  
    /* Remember some variables for next time */
    right_lastInput = pid_right_input;
}