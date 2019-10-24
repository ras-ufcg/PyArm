/*
*
* Código do controlador pra arduino
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define DEBUG       true
#define SERVOMIN    150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX    600 // this is the 'maximum' pulse length count (out of 4096)
#define DOF         5

uint8_t servonum = 0;
char terminator = '\n';
int comprimento = 3, // o correto aqui é 3, mas para os testes com o serialMonitor tem que ser 4 por causa do enter.
    contadorDeComandos = 0;
int comandoCompleto[5];


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
    
    Serial.print("Recebi: ");
    Serial.println(angulo);
    
    Serial.print("Número de comandos recebidos até agora: ");
    Serial.println(contadorDeComandos);

    comandoCompleto[contadorDeComandos++] = angulo;

    if(contadorDeComandos > DOF)
    {
        contadorDeComandos = 0;
        //executaComandos();
    }

    int pulselength = map(angulo, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(15, 0, pulselength);
  }



/*
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(0, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(0, 0, pulselen);
  }

  delay(500);
*/
}
