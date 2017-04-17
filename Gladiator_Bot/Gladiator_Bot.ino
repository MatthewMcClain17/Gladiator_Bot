/* QUIKBOT SHIELD
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2017 Oakwood Robotics
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
const int HBRIDGE_LEFT_EN = 11; // formerly BRIDGE_EN12
const int HBRIDGE_LEFT1 = 12;
const int HBRIDGE_LEFT2 = 13;

const int HBRIDGE_RIGHT_EN = 6;
const int HBRIDGE_RIGHT1 = 5;
const int HBRIDGE_RIGHT2 = 4;

int velocity;
int motorDirection;

byte lastRead[3];
/*  This array stores values from the last recieved serial transmission from the
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

 byte dumpBuffer[30]; // Used to hold corrupted serial information

void setup() {
  /* Joystick (delete when controller sketch is complete)
  pinMode(xPin, INPUT);
  pinMode(yPin, OUTPUT); */
  
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
  // change below to 10
  if (Bluetooth.available() >= 5) { // if any incoming serial data is recieved
    parseBluetoothData(); // decode the message and put the values in lastRead[]
  }

  bluetoothTest();
  
  //delay(50); // Delay to slow transmission/action speed (possibly uneccessary)

  updateDrivingMotors(lastRead[0], lastRead[2]);
  //updateDrivingMotors(map(analogRead(xPin), 0, 1023, 0, 255), map(analogRead(yPin), 0, 1023, 0, 255));
}

// FUNCTIONS

void updateDrivingMotors(int x, int y) {
  // Updates the velocity of motors connected to H-Bridge 1
  // to do: make the Y value do something
  // to do: write a code that could actually make something move using this
  // as a framework
  
  // Left motor: controlled by X-axis position
  
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
  }

  
  // Right motor: controlled by Y-axis position
  
  if (y > (127.5 - deadzone) && y < (127.5 + deadzone)) { // if joystick is near center
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // stop motor
  }
  else if (y >= (127.5 + deadzone)) { // if joystick is pushed up (make sure up = higher #s)
    velocity = map(y, (127.5 + deadzone), 255, 0, 255);

    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_RIGHT1, HIGH);
    digitalWrite(HBRIDGE_RIGHT2, LOW);
    analogWrite(HBRIDGE_RIGHT_EN, velocity); // turn on H-Bridge
  } else if (y <= (127.5 - deadzone)) { // if joystick is pushed down
    velocity = map(y, 0, (127.5 - deadzone), 255, 0);
    
    digitalWrite(HBRIDGE_RIGHT_EN, LOW); // shut off H-Bridge
    digitalWrite(HBRIDGE_RIGHT1, LOW);
    digitalWrite(HBRIDGE_RIGHT2, HIGH);
    analogWrite(HBRIDGE_RIGHT_EN, velocity); // turn on H-Bridge
  }
}

void parseBluetoothData() {
  // Store data recieved via bluetooth in the lastRead array
  if (Bluetooth.read() == ']') { // if first header character ']' is present
    if (Bluetooth.read() == '!') { // if second header character '!' is present
      Bluetooth.readBytes(lastRead, 3); // Store next 3 bytes in lastRead
      // (formerly readBytesUntil())^
    } else { // if initializer is invalid
      Bluetooth.readBytes(dumpBuffer, Serial.available()); // dump entire serial buffer
    }
  } else { // if initializer is invalid
    Bluetooth.readBytes(dumpBuffer, Serial.available()); // dump entire serial buffer
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
