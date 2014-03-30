#include <ros.h> // Timer.h seems to bugger it up, so nope.

// ZumoMotors uses Timer1

#include <ZumoMotors.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

// Compass
#include <Wire.h> // To communicate with compass over i2c
#include <LSM303.h>

#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate


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

// ROS
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;

ZumoMotors motors; // Motor controller

LSM303 compass;

// Compass heading publisher
std_msgs::Float32 compass_heading;
ros::Publisher compass_pub("heading", &compass_heading);

void publishCompassHeading() {
  compass_heading.data = getCompassReading();
  compass_pub.publish( &compass_heading );
}

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

// State Callback
int state = 0; // 0 - safe, 1 - calibrate compass, 2 - normal
std_msgs::Int32 state_msg;
void state_callback( const std_msgs::Int32& state_msg) {
  if (state_msg.data == 1) {
    state = 1;
    calibrateCompass();
    state = 0;
  }
}

// State Subscriber
ros::Subscriber<std_msgs::Int32> state_topic("state", &state_callback);

// Command Velocity Subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);

unsigned long last_battery_published = 0;
////////////////////////////////////////////////////////////////////////////////

void setup()
{  
  pinMode(LED_PIN, OUTPUT);

  // Initialize Compass
  setupCompass();
  
  // Initialize Encoders
  pinMode (ENCODER_LEFT_PIN_A, INPUT);
  pinMode (ENCODER_LEFT_PIN_B, INPUT);
  pinMode (ENCODER_RIGHT_PIN_A, INPUT);
  pinMode (ENCODER_RIGHT_PIN_B, INPUT);

  // Attach Encoder Interrupts
  attachInterrupt(1, measureLeftEncoder, CHANGE); // 1 = pin 3 interrupt
  attachInterrupt(0, measureRightEncoder, CHANGE); // 0 = pin 2 interrupt

  // Left motor power wiring is reversed.
  motors.flipLeftMotor(true);
  
  // Initial speed is zero
  drive(0, 0);

  // Initialize PIDs
  initPIDs();

  nh.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  nh.initNode();
  nh.advertise(battery_pub); // Battery Voltage Publisher
  nh.advertise(compass_pub); // Compass Publisher

  // Encoder publishers
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);

  nh.subscribe(state_topic); // Robot State subscriber
  nh.subscribe(cmd_vel_topic); // Command Velocity Subscriber

  last_battery_published = millis();
}

void loop()
{  
  // Publish battery voltage every second
  if (millis() - last_battery_published > BATTERY_VOLTAGE_PUBLISH_RATE)
  {
    publishBatteryVoltage();
    if (state >= 2) publishCompassHeading();
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

void setupCompass()
{
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate
}


// Will spin in place, overriding cmd_vel etc.
void calibrateCompass()
{
    LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(200);
  motors.setRightSpeed(-200);

  for(unsigned char index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    delay(50);
  }

  // Stop spinning
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  state = 2; // Set to next state
}

// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float getCompassReading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}
