/*************************************************** 
 *  
 *  
 *  
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver m0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver m1 = Adafruit_PWMServoDriver(0x43);
Adafruit_PWMServoDriver m2 = Adafruit_PWMServoDriver(0x44);
Adafruit_PWMServoDriver m3 = Adafruit_PWMServoDriver(0x47);
Adafruit_PWMServoDriver m4 = Adafruit_PWMServoDriver(0x48);
Adafruit_PWMServoDriver m5 = Adafruit_PWMServoDriver(0x51);


#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  m0.begin();
  
  m0.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  // Drive each servo one at a time
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    m0.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    m0.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  servonum ++;
  if (servonum > 7) servonum = 0;
}