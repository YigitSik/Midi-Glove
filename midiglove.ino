#include "midi.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

const int flexPin = A0; //pin A0 to read analog input

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

const byte octaveStep = 12;
byte octave = 60;
byte currentNote;

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  //  Wire.begin();
  //  mpu6050.begin();
  //  mpu6050.calcGyroOffsets(true);
}

void loop() {
  checkFlexSensor();
  checkImu();
}

void checkFlexSensor() {

  value = analogRead(flexPin);
  value = map(value, 700, 900, 0, 500); //Map value 0-1023 to 0-255 (PWM)
 
  
    if (value > 300 && cond==false) {
  
  
      rollAccel = mpu6050.getAccX();
      rollAngle = mpu6050.getAngleX();
      rollGyro = mpu6050.getGyroX();
  
      byte rollValue = map(rollAngle, -40, 60, 0, 11);
  
      currentNote = (findClosestValue(majorScale, majorScaleLength, rollValue)+octave)%127;
  
//       Serial.println("Note on");
//       Serial.println(currentNote);
//  
      noteOn(0x90, currentNote, 0x45);
      delay(500);
      cond = true;
    }
    else if (value <300 && cond==true) {
      noteOn(0x80, currentNote, 0x45);
      cond=false;
      delay(500);
//      Serial.println("note off");
    }

}

int findClosestValue(byte arr[], byte n, byte target) {
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
    delay(1000);
  }
  else if (pitchGyro > 300 ) {
    //    Serial.println("Pitch Down");
    octave = octave - octaveStep;
    delay(1000);
  }

  if (yawGyro > 300) {
    //    Serial.println("Yaw Left");
    delay(1000);
  }
  else if ( yawGyro < -300) {
    //    Serial.println("Yaw Right");
    delay(1000);
  }

  //  if(roll > 1.80 && rollAngle > 45){
  //    Serial.println("Roll Right");
  //    delay(3000);
  //  }
  //  else if(roll <-1.80 && rollAngle <-45){
  //    Serial.println("Roll Left");
  //    delay(3000);
  //  }


}
