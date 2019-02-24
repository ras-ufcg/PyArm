/* Servo Shield Controller
 *  por Lyang Leme de Medeiros - 23/02/2019
 *  
 *  Mapa de ligações no shield
 *  
 *  S1 - D9
 *  S2 - D10
 *  S3 - D11
 *  S4 - D3
 *  S5 - D5
 *  S6 - D6
*/

#include <Servo.h>

#define MIN_PW 500
#define MAX_PW 2400
#define COMP 5

Servo s1, s2, s3, s4, s5, s6;

char rec[5];

int pos = 0;    // variable to store the servo position


void setup() 
{
  Serial.begin(9600);
  
  s1.attach(9, MIN_PW, MAX_PW);
  s2.attach(10, MIN_PW, MAX_PW);
  s3.attach(11, MIN_PW, MAX_PW);
  s4.attach(3, MIN_PW, MAX_PW);
  s5.attach(5, MIN_PW, MAX_PW);
  s6.attach(6, MIN_PW, MAX_PW);
}

void loop() 
{
 if(Serial.available()) 
 {
  Serial.readBytes(rec, COMP);
  cntrlServo(rec);
 }
}

void cntrlServo(char rec[])
{
  char conv[3];
 
  for(int i = 0; i < COMP-2; i++) conv[i] = rec[i+1];
  
  int pos = atoi(conv);
    
  switch(rec[0])
  {
    case '1':
      s1.write(pos);
      break;
    case '2':
      s2.write(pos);
      break;
    case '3':
      s3.write(pos);
      break;
    case '4':
      s4.write(pos);
      break;
    case '5':
      s5.write(pos);
      break;
    case '6':
      s6.write(pos);
      break;
    default:
      //
      break;
  }
}
