#include "midi.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include <Wire.h>
#include <time.h>

const byte flexPin = A0; //pin A0 to read analog input
const byte joystickXPin = A4;
const byte joystickButtonPin = 2;


int value;
byte majorScaleLength = majorScaleLength = sizeof(majorScale) / sizeof(majorScale[0]);;
bool cond = false;

int joystickX;
byte joystickButton;

const byte octaveStep = 11;
byte octave = 60;
byte currentNote;
byte velocityValue;

byte scaleIndex = 0;
byte currentScale[8];

unsigned long start_time;
unsigned long timed_event;
unsigned long current_time;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(9600);
  pinMode(joystickButtonPin, INPUT_PULLUP);
  Wire.begin();
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  accelgyro.initialize();

  timed_event = 500; // after 1000 ms trigger the event
  current_time = millis();
  start_time = current_time;

}

void loop() {

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  checkFlexSensor();
  checkJoystick();
  checkImu();


}


void checkFlexSensor() {

  joystickX = analogRead(joystickXPin);
  value = analogRead(flexPin);
  value = map(value, 700, 900, 0, 200); //Map value 0-1023 to 0-255 (PWM)
  velocityValue = map(joystickX, 0, 1023, 127, 0);

  if (value > 180 && cond == false) {

    byte rollValue = map(ay, -10000, 16000, 0, 11);

    Serial.println(rollValue);

    switch (scaleIndex) {
      case 0:
        currentNote = (findClosestValue(majorScale, majorScaleLength, rollValue) + octave) % 127;
        break;
      case 1:
        currentNote = (findClosestValue(minorScale, majorScaleLength, rollValue) + octave) % 127;
        break;
      case 2:
        currentNote = (findClosestValue(harmonicMinorScale, majorScaleLength, rollValue) + octave) % 127;
        break;
      case 3:
        currentNote = (findClosestValue(phrygianDominantScale, majorScaleLength, rollValue) + octave) % 127;
        break;
      default:
        currentNote = (findClosestValue(majorScale, majorScaleLength, rollValue) + octave) % 127;
    }

    send_midi(NOTE_ON, currentNote, velocityValue);
    cond = true;

  }
  else if (value < 155 && cond == true) {
    send_midi(NOTE_OFF, currentNote, velocityValue);
    cond = false;
  }

}

byte findClosestValue(byte arr[], byte n, byte target) {
  if (target <= arr[0])
    return arr[0];
  if (target >= arr[n - 1])
    return arr[n - 1];

  byte i = 0;
  byte j = n;
  byte mid = 0;
  while (i < j) {
    mid = (i + j) / 2;

    if (arr[mid] == target)
      return arr[mid];

    if (target < arr[mid]) {

      if (mid > 0 && target > arr[mid - 1])
        return getClosest(arr[mid - 1],
                          arr[mid], target);
      j = mid;
    }

    else {
      if (mid < n - 1 && target < arr[mid + 1])
        return getClosest(arr[mid],
                          arr[mid + 1], target);
      // update i
      i = mid + 1;
    }
  }
  return arr[mid];
}

byte getClosest(byte val1, byte val2,
                byte target)
{
  if (target - val1 >= val2 - target)
    return val2;
  else
    return val1;
}


void checkJoystick() {

  bool isJoystickPushed = false;
  int pitchValue;

  if (!digitalRead(joystickButtonPin)) {
    isJoystickPushed = true;
    pitchValue = map(gz , 25000, -25000, 0x00, 0x60);


    if (pitchValue > 0x00 && pitchValue < 0x60) {
      Serial.write(PITCH);
      Serial.write(0x00);
      Serial.write(pitchValue);
    }
  }

  if (isJoystickPushed && digitalRead(joystickButtonPin)) {
    send_midi(NOTE_OFF, currentNote, velocityValue);
  }

}

void checkImu() {

  if ( gx < -10000 && ax > 6000  ) {

    current_time = millis();


    if (current_time - start_time >= timed_event) {
      octave = octave + octaveStep;
      start_time = current_time;
    }

  }
  else if (gx > 10000 && ax < -6000  ) {

    current_time = millis();

    if (current_time - start_time >= timed_event) {
      octave = octave - octaveStep;
      start_time = current_time;
    }

  }

  if (gz > 25000  ) {

    current_time = millis();

    if (current_time - start_time >= timed_event) {
      scaleIndex = (scaleIndex + 1) % 4;
      start_time = current_time;
    }

  }
  else if ( gz < -25000  ) {

    current_time = millis();

    if (current_time - start_time >= timed_event) {
      scaleIndex = (scaleIndex - 1) % 4;
      start_time = current_time;
    }

  }

}
