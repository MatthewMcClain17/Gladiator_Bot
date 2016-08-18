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

const int deadzone = 10;
// Sets the range of joystick values treated as neutral. (0 - 255)

// PINS
// Joystick (delete when controller sketch is complete)
const int xPin = 0;
const int yPin = 1;
const int swPin = 5;

// H-Bridge
// to do: change variable names to HBRIDGE format
const int BRIDGE1_EN12 = 11;
const int BRIDGE1_LEFT1 = 12;
const int BRIDGE1_LEFT2 = 13;

int velocity;
int motorDirection;

byte lastRead[10]; // = {255,255,1,255,255,1,0,1,1}; [delete initializer soon]
/*  This array stores values from the last recieved serial transmission from the
 *  controller in the following format:
 *  
 *  lastRead[0]: Left Joystick X-axis position
 *  lastRead[1]: Left Joystick Y-axis position
 *  lastRead[2]: Left Joystick button state
 *  lastRead[3]: Right Joystick X-axis position
 *  lastRead[4]: Right Joystick Y-axis position
 *  lastRead[5]: Right Joystick button state
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
  pinMode(yPin, OUTPUT);
  
  // H-Bridge
  pinMode(BRIDGE1_EN12, OUTPUT);
  pinMode(BRIDGE1_LEFT1, OUTPUT);
  pinMode(BRIDGE1_LEFT2, OUTPUT);

  // Begin serial communication
  Serial.begin(9600);
  Bluetooth.begin(9600);

  //parseSerialData2(); // [testing, delete]
}

void loop() {
  // change below to 10
  if (Bluetooth.available() >= 11) { // if any incoming serial data is recieved
    //parseSerialData();
    parseSerialData2(); // decode the message and put the values in lastRead[]
  }
  
  for (int i = 0; i <10; i++) { // [for testing purposes]
    Serial.println(lastRead[i]);
  }
  delay(50);
  
  //delay(200);

  // updateDrivingMotors(lastRead[0], lastRead[1]);
  // updateDrivingMotors(map(analogRead(xPin), 0, 1023, 0, 255), analogRead(yPin));
}

// FUNCTIONS

void updateDrivingMotors(int x, int y) {
  // Reads input from joysticks and updates speed of motors connected to
  // H-Bridge 1 appropriately
  // to do: Change values to reflect mapped 1-byte values recieved from serial
  // to do: make the Y value do something
  // to do: write a code that could actually make something move using this
  // as a framework
  
  // determine direction
  if (x > (127.5 - deadzone) && x < (127.5 + deadzone)) { // if joystick is near center
    digitalWrite(BRIDGE1_EN12, LOW); // stop motor
  }
  else if (x >= (127.5 + deadzone)) { // if joystick is pushed right
    velocity = map(x, (127.5 + deadzone), 255, 0, 255);

    digitalWrite(BRIDGE1_EN12, LOW); // shut off H-Bridge
    digitalWrite(BRIDGE1_LEFT1, HIGH);
    digitalWrite(BRIDGE1_LEFT2, LOW);
    analogWrite(BRIDGE1_EN12, velocity); // turn on H-Bridge
  } else { // if joystick is pushed left
    velocity = map(x, 0, (127.5 - deadzone), 255, 0);
    
    digitalWrite(BRIDGE1_EN12, LOW); // shut off H-Bridge
    digitalWrite(BRIDGE1_LEFT1, LOW);
    digitalWrite(BRIDGE1_LEFT2, HIGH);
    analogWrite(BRIDGE1_EN12, velocity); // turn on H-Bridge
  }
}

void parseSerialData() {
  for (int i = 0; i < 10; i++) {
    lastRead[i] = Bluetooth.parseInt();
  }
}

void parseSerialData2() {
  // Potential replacement for parseSerialData using Serial.readBytesUntil()

  if (Bluetooth.read() == '>') {
    Bluetooth.readBytesUntil('>', lastRead, 10);
  }

  /* for testing
  for (int i = 0; i <10; i++) { // [for testing purposes]
    Serial.println(lastRead[i]);
  }
  */
}


// Testing functions (can be deleted to save memory)

void bluetoothTest() {
  // Print collected data to computer's serial monitor
  Serial.print('<');
  for (int n = 0; n < 10; n++) {
    Serial.print(lastRead[n]);
    Serial.print(", ");
  }
  Serial.println('>');
}
