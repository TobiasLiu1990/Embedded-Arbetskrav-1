/*
  Arbetskrav 1 - L293D
  
  Components used:
    L293D
    Motor with fan
    Joystick
    Membrane Switch module
    MAX7219 8x8 Dot matrix LED

  
  References:
    ezButton library - https://github.com/ArduinoGetStarted/button , https://arduinogetstarted.com/tutorials/arduino-button-library
    Keypad Library for Arduino - https://playground.arduino.cc/Code/Keypad/
    LedControl.h - www.elegoo.com
*/

#include <ezButton.h>
#include <Keypad.h>
#include <LedControl.h>


//L293D
const int enablePin1 = 9;  // Pin 1 (1,2EN)
const int pin1A = A2;      // Pin 2 (1A)
const int pin2A = A3;      // Pin 7 (2A)

// Joystick - Button for On/off + X-axis
ezButton buttonPin(13);     //Button object to Pin 8
const int analogPinX = A0;  //X-axis Pin
const int stateOff = 0;
const int stateOn = 1;
int switchState = stateOff;

//Keypad - Using only 4 rows + 3 cols (not using the letters)
const byte rows = 4;
const byte cols = 3;
const byte rowPins[rows] = { 8, 7, 6, 5 };
const byte colPins[cols] = { 4, 3, 2 };
const char keys[rows][cols] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '#', '0', '*' }
};

Keypad pad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);

//MAX7219 8x8 Dot Matrix
const int dataIn = 12;
const int cs = 11;  //load
const int clk = 10;
const int numOfMax7219 = 1;
LedControl lc = LedControl(dataIn, clk, cs, numOfMax7219);

unsigned long delayTime = 250;
unsigned long delayTime2 = 75;

int speed = 100;

void setup() {
  Serial.begin(9600);

  pinMode(enablePin1, OUTPUT);
  pinMode(pin1A, OUTPUT);
  pinMode(pin2A, OUTPUT);

  buttonPin.setDebounceTime(30);  //DebounceTime - Time needed for button to register click.
  lc.shutdown(0, false);          //Start MAX7219
  lc.setIntensity(0, 8);          //Brightness
  lc.clearDisplay(0);
}

void loop() {
  buttonPin.loop();  //button lib uses this to get the state of the button - must have!

  if (buttonPin.isPressed()) {
    if (switchState == stateOff) {
      switchState = stateOn;
      writeOn();  //Matrix writes "ON"
    } else {
      switchState = stateOff;
      lc.clearDisplay(0);
      digitalWrite(pin1A, LOW);
      digitalWrite(pin2A, LOW);
      analogWrite(enablePin1, 0);
    }
  }


  //If motor is on - allow for direction change and speed change via keypad.
  if (switchState == stateOn) {
    switchMotorDirection();  //Direciton

    char key = pad.getKey();  //Enter keypad by pressing *
    enterSpeedByPad(key);

    if (speed == 0) {
      writeOn();
    }
  }
}

void switchMotorDirection() {
  int joystickSensorValue = analogRead(analogPinX) - 495;         //Joystick X-axis default = 495 (+-2). Subtracted by 495 here trying to get 0 as starting point.
  int joystickOutput = map(joystickSensorValue, 0, 1023, 0, 11);  //now default = 0, min = -5, max = 5. Not necesary but nice to have a "normal" starting point.
  //Serial.print("X-axis = ");
  //Serial.println(joystickOutput);

  //Can change the value here for different sensitivity when to activate direction change on joystick.
  if (joystickOutput >= 4) {
    setMotorToClockwise();
    lc.clearDisplay(0);
    writeArrowForward();

  } else if (joystickOutput <= -4) {
    setMotorToCounterClockwise();
    lc.clearDisplay(0);
    writeArrowBackwards();
  }
  setMotorSpeed();
}

void setMotorToClockwise() {
  digitalWrite(pin1A, HIGH);
  digitalWrite(pin2A, LOW);
}

void setMotorToCounterClockwise() {
  digitalWrite(pin1A, LOW);
  digitalWrite(pin2A, HIGH);
}

void enterSpeedByPad(char key) {
  if (key == '#') {
    Serial.println("Keypad started");

    speed = changeSpeedByPad(key);
    updateMotorspeed(speed);

    Serial.print("Current speed is: ");
    Serial.print(speed);
  }
}

void setMotorSpeed() {
  analogWrite(enablePin1, speed);
}

int changeSpeedByPad(char key) {
  String input = "";

  while (key != '*') {  //# = start keypad
    key = pad.getKey();

    if (key) {
      if (key == '#') {
        Serial.println("This does nothing in here");
      } else {
        input += key;
        Serial.print("Key: ");
        Serial.println(key);
      }
    }
  }


  if (input.toInt() > 255) {  //Max speed 255 rpm.
    return 255;
  } else {
    return input.toInt();
  }
}

void updateMotorspeed(int speed) {
  analogWrite(enablePin1, speed);
}

/*
  When the pins are facing horizontally, rows and columns are correct.
  I have connected the led matrix directly into the breadboard, which makes the pins face vertically.
  This means the text customized to this will not make so much sense if the matrix is faced "normally";
*/
void writeOn() {
  byte o[4] = { B01100000, B10010000, B10010000, B01100000 };
  byte n[4] = { B00001001, B00001101, B00001011, B00001001 };
  lc.setColumn(0, 7, o[0]);
  lc.setColumn(0, 6, o[1]);
  lc.setColumn(0, 5, o[2]);
  lc.setColumn(0, 4, o[3]);
  delay(delayTime);
  lc.setColumn(0, 3, n[0]);
  lc.setColumn(0, 2, n[1]);
  lc.setColumn(0, 1, n[2]);
  lc.setColumn(0, 0, n[3]);
}

void writeArrowForward() {
  byte arrowForward[7] = { B00010000, B00001100, B00000110, B01111111, B00000110, B00001100, B00010000 };
  delay(delayTime2);
  lc.setColumn(0, 7, arrowForward[0]);
  lc.setColumn(0, 6, arrowForward[1]);
  lc.setColumn(0, 5, arrowForward[2]);
  lc.setColumn(0, 4, arrowForward[3]);
  lc.setColumn(0, 3, arrowForward[4]);
  lc.setColumn(0, 2, arrowForward[5]);
  lc.setColumn(0, 1, arrowForward[6]);
}

void writeArrowBackwards() {
  byte arrowBackwards[7] = { B00001000, B00110000, B01100000, B11111110, B01100000, B00110000, B00001000 };
  delay(delayTime2);
  lc.setColumn(0, 7, arrowBackwards[0]);
  lc.setColumn(0, 6, arrowBackwards[1]);
  lc.setColumn(0, 5, arrowBackwards[2]);
  lc.setColumn(0, 4, arrowBackwards[3]);
  lc.setColumn(0, 3, arrowBackwards[4]);
  lc.setColumn(0, 2, arrowBackwards[5]);
  lc.setColumn(0, 1, arrowBackwards[6]);
}