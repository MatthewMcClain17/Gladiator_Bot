/* QUIKBOT CONTROLLER
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2016 Oakwood Robotics
 */

// PINS
// Joystick
const int xPin = 0;
const int yPin = 1;
const int swPin = 5;

int x;
int y;

void setup() {
  // Joystick
  pinMode(xPin, INPUT);
  pinMode(yPin, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  joystickTest();
}

void joystickTest() {
  x = analogRead(xPin);
  y = analogRead(yPin);
  Serial.print("X: ");
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.println(""); // adds an extra line of space
  delay(200);
}
