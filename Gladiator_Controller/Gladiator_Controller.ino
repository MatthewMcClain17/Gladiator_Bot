/* QUIKBOT CONTROLLER
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2017 Oakwood Robotics
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

byte positions[3];
/*  This array stores values read from the joysticks and buttons on the
 *  controller in the following format:
 *  
 *  lastRead[0]: Left Joystick Y-axis position
 *  lastRead[1]: Button state
 *  lastRead[2]: Right Joystick X-axis position
 *  
 *  Note that joystick position values range from 0-255 and button state values
 *  should always be either 0 (if not pressed) or 1 (if pressed). The button is
 *  placed between the joystick values to differentiate non-adjacent 3-digit
 *  joystick values from adjacent 3-digit initializer values.
 */

// PINS
// To do: reassign pins to match up with controller designs
const int yPinLeft = 1;
const int xPinRight = 2;
const int button = 8;


void setup() {
  // PIN DECLARATIONS
  // (It is technically unneccessary to declare pins as inputs because all pins
  // are inputs by default – delete any such statements to save memory)

  pinMode(yPinLeft, INPUT);
  pinMode(xPinRight, INPUT);
  pinMode(button, INPUT);
  
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
  
  positions[0] = map(analogRead(yPinLeft), 0, 1023, 0, 255);
  
  if (pressed(button) == HIGH) positions[1] = 1; else positions[1] = 0;
  
  positions[2] = map(analogRead(xPinRight), 0, 1023, 0, 255);
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
  // Prints all three positions[] values to the serial monitor
  for (int i = 0; i < 3; i++) {
    Serial.print(positions[i]);
    Serial.print(',');
  }
  Serial.println();
}

void joystickTest() {
  // Prints the current analog value recieved from the joystick
  // to do: add second joystick
  int x = analogRead(xPinRight);
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

void bluetoothTest(byte data[3]) {
  // Writes any ten-byte array to the Bluetooth serial port
  /*
  for (int i = 0; i < 10; i++) {
    data[i] = 1;
  }*/

  // Print header characters "]!"
  Bluetooth.print(']');
  Bluetooth.print('!');
  Bluetooth.write(data, 3);
  delay(50);

  /*
  for (int i = 0; i < 10; i++) {
    data[i] = 0;
  }
  Bluetooth.print('>');
  Bluetooth.write(data, 10);
  delay(200);*/
}
