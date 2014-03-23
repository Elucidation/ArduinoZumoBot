// Pan/Tilt servo control over serial link
#include <Servo.h> // disables PWM on 9/10
#include <Wire.h>

// Servo pins
#define SERVO_PAN_PIN 5
#define SERVO_TILT_PIN 6

// Pan servo center min/max range (0-180 degrees with 90 degrees centred)
#define SERVO_PAN_MIN 40
#define SERVO_PAN_MAX 140

// Tilt servo center min/max range (0-180 degrees with 90 degrees centred)
#define SERVO_TILT_MIN 50
#define SERVO_TILT_MAX 130

#define SLAVE_ADDRESS 0x04

#define LED_LEFT 3
#define LED_RIGHT 11

// Servo objects
Servo servo_pan;
Servo servo_tilt;

// Servo positions (0-180 degrees with 90 degrees centred)
int pan_pos = 0;
int tilt_pos = 0;


// Smoothly move to pan tilt pos 0-100 range for both (50 centered)
void moveTo(int pan, int tilt)
{
  // Constrain incoming to  0-100 range
  pan = constrain(pan, 0, 100);
  tilt = constrain(tilt, 0, 100);
  while (abs(pan_pos - pan) + abs(tilt_pos - tilt) > 0) {
    pan_pos += pan_pos > pan ? -1 : pan_pos < pan ? 1 : 0;
    tilt_pos += tilt_pos > tilt ? -1 : tilt_pos < tilt ? 1 : 0;
    setPan( map(pan, 0, 100, SERVO_PAN_MIN, SERVO_PAN_MAX) );
    setTilt( map(tilt, 0, 100, SERVO_TILT_MIN, SERVO_TILT_MAX) );
    // Serial.print(pan_pos);
    // Serial.print("|");
    // Serial.println(tilt_pos);
    delay(2); // ms
  }
}

void lightLEDs(int left, int right)
{
  analogWrite(LED_LEFT, map(constrain(left, 0, 100), 0, 100, 0, 255));
  analogWrite(LED_RIGHT, map(constrain(right, 0, 100), 0, 100, 0, 255));
}
// Moves to pan and tilt in range 0 to 100, where 50 is centered
void setPanTilt(int pan, int tilt)
{
  // (pan - 50) + 90
  setPan( map(pan, 0, 100, SERVO_PAN_MIN, SERVO_PAN_MAX) );
  setTilt( map(tilt, 0, 100, SERVO_TILT_MIN, SERVO_TILT_MAX) );
}

void setZero()
{
  // servo_pan.write(90);
  // servo_tilt.write(90);
  setPanTilt(50,50);
}

void setPan(int pan_deg)
{
  servo_pan.write(constrain(pan_deg, SERVO_PAN_MIN, SERVO_PAN_MAX));
}

void setTilt(int tilt_deg)
{
  servo_tilt.write(constrain(tilt_deg, SERVO_TILT_MIN, SERVO_TILT_MAX));
}

void printServoPositions()
{
  Serial.print("Servo Positions (pan/tilt): ");
  Serial.print(pan_pos);
  Serial.print(" ");
  Serial.println(tilt_pos);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.

 Commands:
  s PAN_ANGLE TILT_ANGLE      --- Set Pan Tilt positions in degrees
  p PAN_ANGLE                 --- Set Pan  position in degrees
  t TILT_ANGLE                --- Set Tilt position in degrees
  z                           --- Zero out to default center positions
  i                           --- Info on current pan/tilt servo angles/positions

 */

byte data[3] = {0,0,0};
void serialEvent()
{
  // Always expect 3 bytes (info, pan, tilt)
  while (Serial.available())
  {
    data[0] = Serial.read();
    switch (data[0])
    {
    // s pan-position-degree tilt-position-degrees
    case 's':
    {
      // Parse two ints for pan/tilt
      // s pan tilt
      // Where degrees are zero-centered to camera facing forward
      int pan = Serial.parseInt();
      int tilt = Serial.parseInt();
      moveTo(pan, tilt);
      Serial.flush();
      printServoPositions();
      break;
    }
    case 'm':
    {
      // Read 2 bytes for pan/tilt info
      while (Serial.available() < 2) {delay(1);}
      digitalWrite(13, !digitalRead(13)); // blink led
      // m<byte><byte> for position
      // Where degrees are zero-centered to camera facing forward
      data[1] = Serial.read();
      data[2] = Serial.read();
      moveTo(data[1], data[2]);
      break;
    }
    case 'l':
    {
      // Read 2 bytes for light led info
      while (Serial.available() < 2) {delay(1);}
      digitalWrite(13, !digitalRead(13)); // blink led
      // m<byte><byte> for position
      // Where degrees are zero-centered to camera facing forward
      data[1] = Serial.read();
      data[2] = Serial.read();
      lightLEDs(data[1], data[2]);
      break;
    }
    case 'z':
    {
      // zero out and recenter
      setZero();
      break;
    }

    case 'i':
    {
      // Print current servo positions
      printServoPositions();
      break;
    }
    }
  }
}

void receiveData(int byteCount) {
  // Always expect 3 bytes (command_type, pan, tilt)
  Serial.println(byteCount);
  if (byteCount == 3)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    switch (data[0])
    {
      case 'm': // Pan/Tilt
      moveTo(data[1], data[2]);
      Wire.flush();
      break;

      case 'l':
      lightLEDs(data[1], data[2]);
      break;
      
      case 'z':
      setZero();
      break;
      
      case 'i':
      printServoPositions();
      break;
    }
  }
  else
  {
    // Flush the rest
    Wire.flush();
//    while (Wire.available()) {Wire.read();}
  }

}

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  
  // Initialize serial
  Serial.begin(9600);
  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  // Attach pins
  servo_pan.attach(SERVO_PAN_PIN);
  servo_tilt.attach(SERVO_TILT_PIN);

  Serial.println("Setting pan/tilt to zero center...");
  setZero();
  printServoPositions();
}

void loop()
{
}

// callback for sending data
void sendData(){
    Wire.write(data[0]);
}

