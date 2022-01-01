#define NOTE_ON 0x90
#define NOTE_OFF 0x80
#define CONTROLLER 0xB0
#define INSTRUMENT 0xC0
#define PITCH 0xE0

 byte majorScale[] = {0,2,4,5,7,9,11};
const byte minorScale[] = {0,2,3,5,7,8,10};
const byte harmonicMinorScale[] = {0,2,3,5,7,8,11};
const byte phrygianDominantScale[] = {0,1,4,5,7,8,10};


void send_midi(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);

}
