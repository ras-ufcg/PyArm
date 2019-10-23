#define JOINTS 5

char message[JOINTS*3];
int joints[JOINTS];
int total;
byte bufferSize;
long int message_int;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20000);
}

void loop() {
  bufferSize = Serial.readBytesUntil(255, message, JOINTS*3);

  total = 0;
  for (int i = 0; i < JOINTS; i++){
    joints[i] = (atoi(message) % int (pow(1000,i+1))) / pow(1000,i);
    Serial.write(joints[i]);
    total += joints[i];
  }
  
  //Serial.write(total);
}
