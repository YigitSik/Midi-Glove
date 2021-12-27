#include "midi.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include <Wire.h>
#include <time.h>

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

const byte octaveStep = 11;
byte octave = 60;
byte currentNote;
byte velocityValue;

unsigned long start_time;
unsigned long timed_event;
unsigned long current_time;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(115200);
  pinMode(joystickButtonPin, INPUT);
  Wire.begin();
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  accelgyro.initialize();

  timed_event = 100; // after 1000 ms trigger the event
  current_time = millis();
  start_time = current_time;
}

void loop() {

 accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Serial.println(ax); //-6000 down 10000 up pitch angle
  //       Serial.println(ay); // -10000 left 16000 right roll
  //        Serial.println(az);
  //          Serial.println(gx); // +10000 up -10000 down pitch
  //         delay(200);
  //        Serial.println(gy);
  //     Serial.println(gz); //30000 left -30000 right yaw

//Serial.println(value);

    checkFlexSensor();
    checkJoystick();
    checkImu();

delay(100);
}

//void timer() {
//
//  int milli_seconds = 1000 * 1;
//  current_time = millis(); // update the timer every cycle
//
//  if (current_time - start_time >= timed_event) {
//    //    Serial.println("Timer expired, resetting"); // the event to trigger
//    start_time = current_time;  // reset the timer
//  }
//}

void checkFlexSensor() {

  joystickX = analogRead(joystickXPin);
  Serial.println(joystickX);
  value = analogRead(flexPin);
  value = map(value, 700, 900, 0, 500); //Map value 0-1023 to 0-255 (PWM)
  velocityValue = map(joystickX, 0, 1023, 127, 0);  
  
  

  if (value > 300 && cond == false) {

    byte rollValue = map(ay, -10000, 16000, 0, 11);

    currentNote = (findClosestValue(majorScale, majorScaleLength, rollValue) + octave) % 127;

    send_midi(NOTE_ON, currentNote, velocityValue);
    cond = true;


  }
  else if (value < 300 && cond == true) {
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

  //joystickY = analogRead(joystickYpin);
  //joystickButton = digitalRead(joystickButtonPin);

  //Serial.println(joystickX);
  //Serial.print(joystickY);
  //Serial.println(joystickButton);

//  bool isJoystickPushed = false;
//  int pitchValue;

//  if (!digitalRead(joystickButtonPin)) {
//    isJoystickPushed = true;
//    pitchValue = map(gz , 25000, -25000, 0x00, 0x60);
//
//    Serial.println("sdfdd");
//
//    if (pitchValue > 0x00 && pitchValue < 0x60) {
//      Serial.write(0xE0);
//      Serial.write(0x00);
//      Serial.write(pitchValue);
//      Serial.println("sdfd");
//      delay(20);
//    }
//  }
//
//  if (isJoystickPushed && digitalRead(joystickButtonPin)) {
//    send_midi(NOTE_OFF, currentNote, velocityValue);
//  }

}

void checkImu() {


  if ( gx > 10000 && ax > 10000) { //+0.50 accelx
    octave = octave + octaveStep;
    Serial.println("pitch up");
  }
  else if (gx < -10000 && ax < -6000) { //-0.40 accelx

    octave = octave - octaveStep;
    Serial.println("pitch down");
  }

  if (gz > 25000) { //30
    Serial.println("Yaw Left");
    //    Serial.write(INSTRUMENT );
    //    Serial.write(5);

  }
  else if ( gz < -25000) {
    Serial.println("Yaw Right");
    //    Serial.write(INSTRUMENT );
    //    Serial.write(7);
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


  //    Serial.write(0xB0);
  //    Serial.write(0x07); // VOLUME
  //    Serial.write(index);


}
