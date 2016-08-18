/* QUIKBOT CONTROLLER
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2016 Oakwood Robotics
 * Slave module address: +ADDR:2016:3:220126
 * Master module address: 
 */

#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(12, 13);
// Creates a new serial port for communication via bluetooth module.
// This makes it so the Arduino's serial port remains unblocked so bluetooth
// and USB communication can take place simultaneously. Parameters: (RX, TX)
// To do: change to lower pins so as not to accidentally turn on onboard LED

// USER SETTINGS
// Select the variable values that best suit your needs. (Ranges in parenthesis)

const int debounce = 5; // number of milliseconds to wait when debouncing buttons

byte positions[10];
/*  This array stores values read from the joysticks and buttons on the
 *  controller in the following format:
 *  // To do: move joystick button states to before axis positions
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

// PINS
// Left Joystick
const int xPinLeft = 0;
const int yPinLeft = 1;
const int joyButtonLeft = 6; // activated by pressing down on left joystick

// Right Joystick
const int xPinRight = 2;
const int yPinRight = 3;
const int joyButtonRight = 7; // activated by pressing down on right joystick

// Buttons
const int button1 = 8;
const int button2 = 9;
const int button3 = 10;
const int button4 = 11;


void setup() {
  // PIN DECLARATIONS
  // (It is technically unneccessary to declare pins as inputs because all pins
  // are inputs by default – delete any such statements to save memory)

  // Left Joystick
  pinMode(xPinLeft, INPUT);
  pinMode(yPinLeft, INPUT);
  pinMode(joyButtonLeft, INPUT);
  
  // Right Joystick
  pinMode(xPinRight, INPUT);
  pinMode(yPinRight, INPUT);
  pinMode(joyButtonRight, INPUT);
  
  // Buttons
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);

  
  Serial.begin(9600); // Begin serial communication at a baud rate of 9600
  Bluetooth.begin(9600);
}

void loop() {
  // Collect input data from joysticks and buttons, then store in positions array
  updatePositions();

  // Testing functions - to delete
  bluetoothTest(positions);
}



// FUNCTIONS

void updatePositions() {
  // Collects information from joysticks and buttons, maps the joystick values
  // so they can fit in a single byte (0-255) and writes it to positions[].
  
  // Left joystick
  positions[0] = map(analogRead(xPinLeft), 0, 1023, 0, 255);
  positions[1] = map(analogRead(yPinLeft), 0, 1023, 0, 255);
  if (pressed(joyButtonLeft) == HIGH) positions[2] = 1; else positions[2] = 0;

  // Right joystick
  positions[3] = map(analogRead(xPinRight), 0, 1023, 0, 255);
  positions[4] = map(analogRead(yPinRight), 0, 1023, 0, 255);
  if (pressed(joyButtonRight) == HIGH) positions[5] = 1; else positions[5] = 0;

  // Buttons
  if (pressed(button1) == HIGH) positions[6] = 1; else positions[6] = 0;
  if (pressed(button2) == HIGH) positions[7] = 1; else positions[7] = 0;
  if (pressed(button3) == HIGH) positions[8] = 1; else positions[8] = 0;
  if (pressed(button4) == HIGH) positions[9] = 1; else positions[9] = 0;
}

bool pressed(int buttonPin) { // to do: rewrite so this is for single presses
  // Checks to see if a button connected to a specific pin (its parameter) has
  // been pressed. Includes a debounce for accuracy. Returns HIGH if pressed,
  // returns LOW if not pressed.
  
  if (digitalRead(buttonPin) == HIGH) {
    delay(debounce);
    if (digitalRead(buttonPin) == HIGH) {
      return HIGH;
    } else return LOW;
  } else return LOW;
}

// Testing functions (can be deleted to save memory)

void printPositions() {
  // Prints all ten positions[] values to the serial monitor
  for (int i = 0; i < 10; i++) {
    Serial.print(positions[i]);
    Serial.print(',');
  }
  Serial.println();
}

void joystickTest() {
  // Prints the current analog value recieved from the joystick
  // to do: add second joystick
  int x = analogRead(xPinLeft);
  int y = analogRead(yPinLeft);
  Serial.print("X: ");
  Serial.println(x);
  Serial.print("Y: ");
  Serial.println(y);
  Serial.println(""); // adds an extra line of space
  delay(200);
}

void buttonTest(int buttonPin) {
  // Prints "Button [number of button] pressed!" to serial monitor; button to
  // test set with parameter, such as buttonTest(button1);
  if (pressed(buttonPin) == HIGH) {
    Serial.print("Button ");
    Serial.print(buttonPin);
    Serial.println(" pressed!");
    delay(100);
  }
}

void bluetoothTest(byte data[10]) {
  // Writes any ten-byte array to the bluetooth serial port
  /*
  for (int i = 0; i < 10; i++) {
    data[i] = 1;
  }*/
  Bluetooth.print('>');
  Bluetooth.write(data, 10);
  delay(50);

  /*
  for (int i = 0; i < 10; i++) {
    data[i] = 0;
  }
  Bluetooth.print('>');
  Bluetooth.write(data, 10);
  delay(200);*/
}
