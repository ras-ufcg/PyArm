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
int aux_pos[6]; // valores de posição antigos

void setup() 
{
  Serial.begin(9600);

  TIMSK0 = 0;
  
  s1.attach(9, MIN_PW, MAX_PW);
  s2.attach(10, MIN_PW, MAX_PW);
  s3.attach(11, MIN_PW, MAX_PW);
  s4.attach(3, MIN_PW, MAX_PW);
  s5.attach(5, MIN_PW, MAX_PW);
  s6.attach(6, MIN_PW, MAX_PW);

  aux_pos[0] = s1.read();
  aux_pos[1] = s2.read();
  aux_pos[2] = s3.read();
  aux_pos[3] = s4.read();
  aux_pos[4] = s5.read();
  aux_pos[5] = s6.read();
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
  int aux;
  char conv[3];
 
  for(int i = 0; i < COMP-2; i++) conv[i] = rec[i+1];

  int pos = atoi(conv);
  int n_atual = atoi(rec[0]);
  Serial.println(n_atual);

  Serial.println(pos);
  Serial.println(aux_pos[n_atual]);
  Serial.println(abs(pos - aux_pos[n_atual]));
  for(int i = 0; i <= abs(pos - aux_pos[n_atual]) ; i++)
  {
    if (pos > aux_pos[n_atual]) 
      aux = i;
    else
      aux = -i;
  
   switch(rec[0])
  {
    case '1':
      s1.write(aux_pos[0] + aux);
      Serial.print("Soma: ");
      Serial.println(aux_pos[0] + aux);
      break;
    case '2':
      s2.write(aux_pos[1] + aux);
      Serial.print("Soma: ");
      Serial.println(aux_pos[0] + aux);
      break;
    case '3':
      s3.write(aux_pos[2] + aux);
      break;
    case '4':
      s4.write(aux_pos[3] + aux);
      break;
    case '5':
      s5.write(aux_pos[4] + aux);
      break;
    case '6':
      s6.write(aux_pos[5] + aux);
      break;
    default:
      
      break;
  }
    
  delay(15);
  }

  aux_pos[n_atual] = pos;
}
