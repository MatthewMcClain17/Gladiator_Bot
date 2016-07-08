/* QUIKBOT CONTROLLER
 * 
 * Written by Matthew McClain, Oakwood Robotics Co-President
 * Copyright 2016 Oakwood Robotics
 */

// USER SETTINGS
// Select the variable values that best suit your needs. (Ranges in parenthesis)

const int debounce = 5;

// PINS
// Joystick
const int xPin = 0;
const int yPin = 1;
const int swPin = 6;

// Buttons
const int button1 = 8;
const int button2 = 9;
const int button3 = 10;
const int button4 = 11;

int x;
int y;

void setup() {
  // Joystick
  pinMode(xPin, INPUT);
  pinMode(yPin, OUTPUT);

  // Buttons (all inputs, so technically unnecessary to declare as such)
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  //joystickTest();
  buttonTest(button4);
}

bool pressed(int buttonPin) {
  if (digitalRead(buttonPin) == HIGH) {
    delay(debounce);
    if (digitalRead(buttonPin) == HIGH) {
      return HIGH;
    } else return LOW;
  } else return LOW;
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

void buttonTest(int button) {
  if (pressed(button) == HIGH) {
    Serial.print("Button ");
    Serial.print(button);
    Serial.println(" Pressed!");
    delay(100);
  }
}
