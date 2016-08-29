/* QUIKBOT SHIELD
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2016 Oakwood Robotics
 */

#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(7, 8);
// Creates a new serial port for communication via bluetooth module.
// This makes it so the Arduino's serial port remains unblocked so bluetooth
// and USB communication can take place simultaneously. Parameters: (RX, TX)


// USER SETTINGS
// Select the variable values that best suit your needs. (Ranges in parenthesis)

const int deadzone = 10; // Sets the range of joystick values treated as neutral. (0 - 255)
const int MAX_SPEED = 255; // Sets the maximum power to send to the motors. (0-255)

// PINS
// Joystick (delete when controller sketch is complete)
const int xPin = 0;
const int yPin = 1;
const int swPin = 5;

// H-Bridge
// to do: change variable names to HBRIDGE format
const int HBRIDGE_LEFT_EN = 11; // formerly BRIDGE_EN12
const int HBRIDGE_LEFT1 = 12;
const int HBRIDGE_LEFT2 = 13;

const int HBRIDGE_RIGHT_EN = 6;
const int HBRIDGE_RIGHT1 = 5;
const int HBRIDGE_RIGHT2 = 4;

int velocity;
int motorDirection;

byte lastRead[10];
/*  This array stores values from the last recieved serial transmission from the
 *  controller in the following format:
 *  
 *  lastRead[0]: Left Joystick button state
 *  lastRead[1]: Left Joystick X-axis position
 *  lastRead[2]: Left Joystick Y-axis position 
 *  lastRead[3]: Right Joystick button state
 *  lastRead[4]: Right Joystick X-axis position
 *  lastRead[5]: Right Joystick Y-axis position
 *  lastRead[6]: Button 1 state
 *  lastRead[7]: Button 2 state
 *  lastRead[8]: Button 3 state
 *  lastRead[9]: Button 4 state
 *  
 *  Note that joystick position values range from 0-255 and button state values
 *  should always be either 0 (if not pressed) or 1 (if pressed).
 */

void setup() {
  // Joystick (delete when controller sketch is complete)
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  
  // H-Bridge
  pinMode(HBRIDGE_LEFT_EN, OUTPUT);
  pinMode(HBRIDGE_LEFT1, OUTPUT);
  pinMode(HBRIDGE_LEFT2, OUTPUT);

  pinMode(HBRIDGE_RIGHT_EN, OUTPUT);
  pinMode(HBRIDGE_RIGHT1, OUTPUT);
  pinMode(HBRIDGE_RIGHT2, OUTPUT);

  // Begin serial communication
  Serial.begin(9600);
  Bluetooth.begin(9600);
}

void loop() {
  /* change below to 10
  if (Bluetooth.available() >= 11) { // if any incoming serial data is recieved
    parseBluetoothData(); // decode the message and put the values in lastRead[]
  }
  */
  //bluetoothTest();
  
  //delay(50); // Delay to slow transmission/action speed (possibly uneccessary

  determineVelocity(map(analogRead(xPin), 0, 1023, 0, 255), map(analogRead(yPin), 0, 1023, 0, 255));
  //updateDrivingMotors(lastRead[1], lastRead[2]);
  //updateLeftMotor(map(analogRead(xPin), 0, 1023, 0, 255));
  //updateRightMotor(map(analogRead(yPin), 0, 1023, 0, 255));
  //updateDrivingMotors(map(analogRead(xPin), 0, 1023, 0, 255), map(analogRead(yPin), 0, 1023, 0, 255));
}

// FUNCTIONS

void updateDrivingMotors(int x, int y) {
  // Updates the velocity of motors connected to H-Bridge 1
  // to do: make the Y value do something
  // to do: write a code that could actually make something move using this
  // as a framework

  // Attach positive motor wires to front of shield (and H-Bridge)

  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  
  // Left motor: controlled by X-axis position
  
  if (x > (127.5 - deadzone) && x < (127.5 + deadzone)) { // if joystick is near center
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // stop motor
  } else if (x >= (127.5 + deadzone)) { // if joystick is pushed right
    velocity = map(x, (127.5 + deadzone), 255, 0, 255);
    // make motor go forwards
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_LEFT1, LOW);
    digitalWrite(HBRIDGE_LEFT2, HIGH);
    analogWrite(HBRIDGE_LEFT_EN, velocity); // turn on H-Bridge
  } else if (x <= (127.5 - deadzone)) { // if joystick is pushed left
    velocity = map(x, 0, (127.5 - deadzone), 255, 0);
    // make motor go backwards
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_LEFT1, HIGH);
    digitalWrite(HBRIDGE_LEFT2, LOW);
    analogWrite(HBRIDGE_LEFT_EN, velocity); // turn on H-Bridge
  }

  
  // Right motor: controlled by Y-axis position
  
  if (y > (127.5 - deadzone) && y < (127.5 + deadzone)) { // if joystick is near center
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // stop motor
  } else if (y >= (127.5 + deadzone)) { // if joystick is pushed down
    velocity = map(y, 0, (127.5 - deadzone), 255, 0);
    // make motor go forwards
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_RIGHT1, LOW);
    digitalWrite(HBRIDGE_RIGHT2, HIGH);
    analogWrite(HBRIDGE_RIGHT_EN, velocity); // turn on H-Bridge
  } else if (y <= (127.5 - deadzone)) { // if joystick is pushed up (lower #s)
    velocity = map(y, (127.5 + deadzone), 255, 0, 255);
    // make motor go backwards
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_RIGHT1, HIGH);
    digitalWrite(HBRIDGE_RIGHT2, LOW);
    analogWrite(HBRIDGE_RIGHT_EN, velocity); // turn on H-Bridge
  }
}

void determineVelocity(byte JOYSTICK_X, byte JOYSTICK_Y) {
  // Determines the appropriate velocities for each motor based on the position
  // of the joysticks on the controller. Uses the Y axis of the left joystick
  // and the X axis of the right joystick.
  // Note: Positive values indicate forward and right directions, negative
  // values indicate backward and left directions.

  // Determine raw speed
  
  int RAW_SPEED; // stores the basic speed and forwards/backwards direction

  if (JOYSTICK_Y > (127.5 - deadzone) && JOYSTICK_Y < (127.5 + deadzone)) { // if joystick is near center
    RAW_SPEED = 0;
  } else if (JOYSTICK_Y >= (127.5 + deadzone)) { // if joystick is pushed down
    RAW_SPEED = map(JOYSTICK_Y, 0, (127.5 - deadzone), -MAX_SPEED, 0);
  } else if (JOYSTICK_Y <= (127.5 - deadzone)) { // if joystick is pushed up (lower #s)
    RAW_SPEED = map(JOYSTICK_Y, (127.5 + deadzone), 255, 0, MAX_SPEED);
  }
  
  // Determine turning magnitude

  float TURNING_MAGNITUDE; // stores the basic turning percentage and direction

  if (JOYSTICK_X > (127.5 - deadzone) && JOYSTICK_X < (127.5 + deadzone)) { // if joystick is near center
    TURNING_MAGNITUDE = 0;
  } else if (JOYSTICK_X >= (127.5 + deadzone)) { // if joystick is pushed right
    TURNING_MAGNITUDE = 1 - map(JOYSTICK_X, (127.5 + deadzone), 255, 0, 1);
  } else if (JOYSTICK_X <= (127.5 - deadzone)) { // if joystick is pushed left
    TURNING_MAGNITUDE = -1 - map(JOYSTICK_X, 0, (127.5 - deadzone), -1, 0);
  }

  // Determine appropriate velocities for left and right motors
  
  int LEFT_VELOCITY; // stores the appropriate velocity for the left motor
  int RIGHT_VELOCITY; // stores the appropriate velocity for the right motor

  if (TURNING_MAGNITUDE > 0) { // if turning right
    LEFT_VELOCITY = RAW_SPEED;
    RIGHT_VELOCITY = RAW_SPEED * TURNING_MAGNITUDE;
  } else if (TURNING_MAGNITUDE < 0) { // if turning left
    LEFT_VELOCITY = RAW_SPEED * -TURNING_MAGNITUDE;
    RIGHT_VELOCITY = RAW_SPEED;
  }

  Serial.print("Raw speed: ");
  Serial.print(RAW_SPEED);
  Serial.print("   Turning magnitude: ");
  Serial.print(TURNING_MAGNITUDE);
  Serial.print("   L and R velocities: ");
  Serial.print(LEFT_VELOCITY);
  Serial.print(' ');
  Serial.println(RIGHT_VELOCITY);

  updateLeftMotor(LEFT_VELOCITY);
  updateRightMotor(RIGHT_VELOCITY);
}

void updateLeftMotor(int VELOCITY) {
  // Sets the speed and direction of a motor attached to the left side
  // of the H-Bridge

  if (VELOCITY == 0) { // if velocity is zero
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // stop motor
  } else if (VELOCITY > 1) { // if velocity is positive
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // turn off H-Bridge
    
    digitalWrite(HBRIDGE_LEFT1, LOW);
    digitalWrite(HBRIDGE_LEFT2, HIGH);
    
    analogWrite(HBRIDGE_LEFT_EN, VELOCITY); // turn on H-Bridge
  } else if (VELOCITY < 0) { // if velocity is negative
    VELOCITY = -VELOCITY; // make velocity positive
    
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // turn off H-Bridge
    
    digitalWrite(HBRIDGE_LEFT1, HIGH);
    digitalWrite(HBRIDGE_LEFT2, LOW);
    
    analogWrite(HBRIDGE_LEFT_EN, VELOCITY); // turn on H-Bridge
  }
}

void updateRightMotor(int VELOCITY) {
  // Sets the speed and direction of a motor attached to the right side
  // of the H-Bridge

  if (VELOCITY == 0) { // if velocity is zero
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // stop motor
  } else if (VELOCITY > 1) { // if velocity is positive
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // turn off H-Bridge
    
    digitalWrite(HBRIDGE_RIGHT1, LOW);
    digitalWrite(HBRIDGE_RIGHT2, HIGH);
    
    analogWrite(HBRIDGE_RIGHT_EN, VELOCITY); // turn on H-Bridge
  } else if (VELOCITY < 0) { // if velocity is negative
    VELOCITY = -VELOCITY; // make velocity positive
    
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // turn off H-Bridge
    
    digitalWrite(HBRIDGE_RIGHT1, HIGH);
    digitalWrite(HBRIDGE_RIGHT2, LOW);
    
    analogWrite(HBRIDGE_RIGHT_EN, VELOCITY); // turn on H-Bridge
  }

  /*
  if (x > (127.5 - deadzone) && x < (127.5 + deadzone)) { // if joystick is near center
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // stop motor
  }
  else if (x >= (127.5 + deadzone)) { // if joystick is pushed right
    velocity = map(x, (127.5 + deadzone), 255, 0, 255);

    digitalWrite(HBRIDGE_LEFT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_LEFT1, HIGH);
    digitalWrite(HBRIDGE_LEFT2, LOW);
    analogWrite(HBRIDGE_LEFT_EN, velocity); // turn on H-Bridge
  } else if (x <= (127.5 - deadzone)) { // if joystick is pushed left
    velocity = map(x, 0, (127.5 - deadzone), 255, 0);
    
    digitalWrite(HBRIDGE_LEFT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_LEFT1, LOW);
    digitalWrite(HBRIDGE_LEFT2, HIGH);
    analogWrite(HBRIDGE_LEFT_EN, velocity); // turn on H-Bridge
  }*/
}

void parseBluetoothData() {
  // Store data recieved via bluetooth in the lastRead array
  if (Bluetooth.read() == '>') { // if header character '>' is present
    Bluetooth.readBytesUntil('>', lastRead, 10); // Store 10 bytes in lastRead
  }
}


// Testing functions (can be deleted to save memory)

void bluetoothTest() {
  // Print collected data to computer serial monitor
  Serial.print('>');
  for (int i = 0; i < 10; i++) {
    Serial.print(lastRead[i]);
    if (i < 9) {
      Serial.print(", ");
    }
  }
  Serial.println();
}
