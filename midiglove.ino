#include "midi.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

const byte flexPin = A0; //pin A0 to read analog input
const byte joystickXPin = A4;
const byte joystickYpin = A5;
const byte joystickButtonPin = 2;

int value;
byte majorScaleLength = majorScaleLength = sizeof(majorScale) / sizeof(majorScale[0]);;
bool cond = false;

float pitchGyro;
float pitchAccel;
float pitchAngle;

float rollGyro;
float rollAccel;
float rollAngle;

float yawGyro;
float yawAccel;
float yawAngle;

float joystickX;
float joystickY;
byte joystickButton;

const byte octaveStep = 12;
byte octave = 60;
byte currentNote;
//byte oldNote;

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  pinMode(joystickButtonPin, INPUT_PULLUP);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  checkFlexSensor();
  checkImu();
  checkJoystick();
}

void checkFlexSensor() {

  value = analogRead(flexPin);
  value = map(value, 700, 900, 0, 500); //Map value 0-1023 to 0-255 (PWM)


  if (value > 300 && cond == false) {


    rollAccel = mpu6050.getAccX();
    rollAngle = mpu6050.getAngleX();
    rollGyro = mpu6050.getGyroX();

    byte rollValue = map(rollAngle, -40, 60, 0, 11);

    currentNote = (findClosestValue(majorScale, majorScaleLength, rollValue) + octave) % 127;

    send_midi(NOTE_ON, currentNote, 0x45);
    cond = true;


  }
  else if (value < 300 && cond == true) {
    send_midi(NOTE_OFF, currentNote, 0x45);
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

  joystickX = analogRead(joystickXPin);
  joystickY = analogRead(joystickYpin);
  joystickButton = digitalRead(joystickButtonPin);

  //Serial.print(joystickX);
  //Serial.print(joystickY);
  //Serial.println(joystickButton);

  bool isJoystickPushed = false;

  int pitchValue;

  if (!digitalRead(joystickButtonPin)) {
    isJoystickPushed = true;
    pitchValue = mpu6050.getGyroZ();
    pitchValue = map(pitchValue , 200, -200, 0x00, 0x60);

    if (pitchValue > 0x00 && pitchValue < 0x60) {
      Serial.write(0xE0);
      Serial.write(0x00);
      Serial.write(pitchValue);
      delay(50);
    }

  }

  if ( isJoystickPushed && digitalRead(joystickButtonPin)) {
    send_midi(NOTE_OFF, currentNote, 0x45);
  }

}

void checkImu() {
  mpu6050.update();

  pitchGyro = mpu6050.getGyroY();
  pitchAccel = mpu6050.getAccAngleY();
  pitchAngle = mpu6050.getAngleY();

  yawAccel = mpu6050.getAccZ();
  yawGyro = mpu6050.getGyroZ();
  yawAngle = mpu6050.getAngleZ();


  if ( pitchGyro < -300) {
    //    Serial.println("Pitch Up");
    octave = octave + octaveStep;
  }
  else if (pitchGyro > 300 ) {
    //    Serial.println("Pitch Down");
    octave = octave - octaveStep;
  }

  if (yawGyro > 300) {
    //    Serial.println("Yaw Left");
  }
  else if ( yawGyro < -300) {
    //    Serial.println("Yaw Right");
  }

  //  if(roll > 1.80 && rollAngle > 45){
  //    Serial.println("Roll Right");
  //    delay(3000);
  //  }
  //  else if(roll <-1.80 && rollAngle <-45){
  //    Serial.println("Roll Left");
  //    delay(3000);
  //  }

  //    Serial.write(0xB0);
  //    Serial.write(0x00);
  //    Serial.write(0x01);
  //
  //    Serial.write(0xB0);
  //    Serial.write(0x20);
  //    Serial.write(0x01);
  //
  //    Serial.write(INSTRUMENT );
  //    Serial.write(index);


  //    Serial.write(0xB0);
  //    Serial.write(0x07); // VOLUME
  //    Serial.write(index);


}
