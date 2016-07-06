/* QUIKBOT SHIELD
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2016 Oakwood Robotics
 */

// PINS
// Joystick (delete when controller sketch is complete)
const int xPin = 0;
const int yPin = 1;
const int swPin = 5;

// H-Bridge 1
const int BRIDGE1_EN12 = 11;
const int BRIDGE1_LEFT1 = 12;
const int BRIDGE1_LEFT2 = 13;

// H-Bridge 2
const int BRIDGE2_EN12 = 6;
const int BRIDGE2_LEFT1 = 7;
const int BRIDGE2_LEFT2 = 8;

int deadzone = 170;

int velocity;
int motorDirection;

void setup() {
  // Joystick (delete when controller sketch is complete)
  pinMode(xPin, INPUT);
  pinMode(yPin, OUTPUT);
  
  // H-Bridge 1
  pinMode(BRIDGE1_EN12, OUTPUT);
  pinMode(BRIDGE1_LEFT1, OUTPUT);
  pinMode(BRIDGE1_LEFT2, OUTPUT);

  // H-Bridge 2
  pinMode(BRIDGE2_EN12, OUTPUT);
  pinMode(BRIDGE2_LEFT1, OUTPUT);
  pinMode(BRIDGE2_LEFT2, OUTPUT);
}

void loop() {
  updateDrivingMotors(analogRead(xPin), analogRead(yPin));
}

// FUNCTIONS

void updateDrivingMotors(int x, int y) {
  // Reads input from joysticks and updates speed of motors connected to
  // H-Bridge 1 appropriately
  
  // determine direction
  if (x > (512 - deadzone) && x < (512 + deadzone)) { // if joystick is near center
    digitalWrite (BRIDGE1_EN12, LOW); // stop motor
  }
  else if (x >= (512 + deadzone)) { // if joystick is pushed right
    velocity = map(x, (512 + deadzone), 1023, 0, 255);
    
    digitalWrite(BRIDGE1_LEFT1, HIGH);
    digitalWrite(BRIDGE1_LEFT2, LOW);
    analogWrite(BRIDGE1_EN12, velocity); // turn on H-Bridge
  } else { // if joystick is pushed left
    velocity = map(x, 0, (511 - deadzone), 255, 0);
    digitalWrite(BRIDGE1_EN12, LOW); // shut off H-Bridge
    
    digitalWrite(BRIDGE1_LEFT1, LOW);
    digitalWrite(BRIDGE1_LEFT2, HIGH);
    analogWrite(BRIDGE1_EN12, velocity); // turn on H-Bridge
  }
}
