#define JOINTS 5

byte message[JOINTS];
int joints[JOINTS];
int total;
byte bufferSize;
long int message_int;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20000);
}

void loop() {
  bufferSize = Serial.readBytesUntil(255, message, JOINTS);

  total = 0;

  for (int i = 0; i < JOINTS; i++){
    joints[i] = message[i];
    Serial.write(joints[i]);
  }
}
