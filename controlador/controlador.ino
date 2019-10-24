/*
*
* Código do controlador pra arduino
* Autor: Lyang Leme de Medeiros - 23/10/2019
* 
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define DEBUG       true
#define SERVOMIN    150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX    600 // this is the 'maximum' pulse length count (out of 4096)
#define DOF         5

char terminator = '\n';

int comprimento = 3,
    contadorDeComandos = 0,
    comandoCompleto[DOF];


void setup() {
  Serial.begin(9600);
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {

  if(Serial.available())
  {
    String entrada = Serial.readStringUntil(terminator);
    int angulo = entrada.toInt();
    
    if(DEBUG)Serial.print("RECEBI: ");
    if(DEBUG)Serial.println(angulo);
    
    comandoCompleto[contadorDeComandos++] = angulo;

    if(DEBUG)Serial.print("Número de comandos recebidos até agora: ");
    if(DEBUG)Serial.println(contadorDeComandos);

  }

  if(contadorDeComandos >= DOF)
    {
        contadorDeComandos = 0;
        executaComandos();
    }

}


void executaComandos()
{
  for(int i=0; i<DOF; i++)
  {
    if(DEBUG)Serial.print("EXECUTANDO: ");
    if(DEBUG)Serial.println(comandoCompleto[i]);
    
    int pulselength = map(comandoCompleto[i], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(i, 0, pulselength);
    delay(1000);
  }
}
