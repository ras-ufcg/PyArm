/*
*
* Código do controlador pra arduino
* Autor: Lyang Leme de Medeiros - 23/10/2019
* 
* SCL - PINO A5
* SDA - PINO A4
* 
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define DEBUG       true // if(DEBUG) Serial.println();
#define SERVOMIN    150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX    600 // this is the 'maximum' pulse length count (out of 4096)
#define DOF         5   // 
#define PASSO       3
#define ENDINICIAL  8   // Endereço do motor da base na PCA9685

int pose[DOF] =  {90, 90, 90, 90, 90};

char terminator = '\n';

int comprimento = 3,
    contadorDeComandos = 0,
    comandoCompleto[DOF];


void setup() {
  Serial.begin(9600);
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);

  poseInicial();
  fechaGarra();

  if(DEBUG) Serial.println("Controlador do Braço Pick and Place");
  if(DEBUG) Serial.println("Status: Pronto!");
}

void loop() {

  if(Serial.available())
  {
    String entrada = Serial.readStringUntil(terminator);

    if(entrada == "00") poseInicial();
    else if(entrada == ".")   pwm.setPWM(12, 0, map(0, 0, 180, SERVOMIN, SERVOMAX));
    else if(entrada == "..")  pwm.setPWM(12, 0, map(90, 0, 180, SERVOMIN, SERVOMAX));
    else if(entrada == "...") pwm.setPWM(12, 0, map(180, 0, 180, SERVOMIN, SERVOMAX));
    else if(entrada == "*")   pwm.setPWM(12, 0, map(45, 0, 180, SERVOMIN, SERVOMAX));
    else if(entrada == "fecha") fechaGarra();
    else if(entrada == "abre") abreGarra();
    else if(entrada == "pega") pegaFita();
    else if(entrada == "volta") poseInicial();
    else if(entrada == "b")
    {
      Serial.println("Simbora");
      poseInicial();
      pegaFita();
      poseInicial();
      abreGarra();
    }
    else montarComando(entrada.toInt());
  }
}

void montarComando(int angulo)
{
  if(DEBUG)Serial.print("RECEBI: ");
  if(DEBUG)Serial.println(angulo);
  
  comandoCompleto[contadorDeComandos++] = angulo;

  if(DEBUG)Serial.print("Número de comandos recebidos até agora: ");
  if(DEBUG)Serial.println(contadorDeComandos);

  
  if(contadorDeComandos >= DOF)
    {
        contadorDeComandos = 0;
        executaComandos();
    }
}

void executaComandos()
{
  // O braço está em "pose" e deve ir para "comandoCompleto"
  // Calcular a diferença entre os angulos de cada motor
  int deltaAngulos [DOF];
  for(int i=0; i<DOF; i++)
  {
    deltaAngulos[i] = comandoCompleto[i] - pose[i];
    
    if(DEBUG) Serial.print("Delta do motor ");
    if(DEBUG) Serial.print(i);
    if(DEBUG) Serial.print(": ");
    if(DEBUG) Serial.println(deltaAngulos[i]);
    
  }

  // Calcular matriz e incrementos
  int incrementos[180/PASSO][DOF]; // [linhas]x[colunas] = [incrementos][motores]
  for(int i=0; i<DOF; i++)
  {
    // Calcular quantas linhas de icremento serão nessesárias pra realizar esse movimento
    int n_incrementos = deltaAngulos[i]/PASSO;    
    int resto_inc = deltaAngulos[i]%PASSO;

    if(DEBUG) Serial.print("Número de incrementos necessários para o motor ");
    if(DEBUG) Serial.print(i);
    if(DEBUG) Serial.print(": ");
    if(DEBUG) Serial.print(n_incrementos);
    if(DEBUG) Serial.print("\tResto: ");
    if(DEBUG) Serial.println(resto_inc);
    
    for(int j=0; j<(180/PASSO); j++)
    {      
      incrementos[j][i] = (j<abs(n_incrementos))?  (n_incrementos/abs(n_incrementos))*PASSO : 0;
    }
    
    incrementos[n_incrementos][i] = resto_inc;
  }
  if(DEBUG){
    Serial.println("INCREMENTOS:");
    for(int i=0; i<(180/PASSO); i++) 
    {
      Serial.print(i);
      Serial.print(": ");
      for(int j=0; j<DOF; j++)
      { 
        Serial.print(incrementos[i][j]);
        Serial.print("\t");
      }
      Serial.println();
    }
  }
  
  for(int i=0; i<DOF; i++)
  {
    if(DEBUG)Serial.print("EXECUTANDO: ");
    if(DEBUG)Serial.println(comandoCompleto[i]);
    
    int pulselength = map(comandoCompleto[i], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(ENDINICIAL+i, 0, pulselength);

    
    
   // pose = comandoCompleto;                                                   //Atualiza a pose atual
    
    delay(1000);
  }

 
}

void poseInicial()
{
  delay(1000);
  for(int i=0; i<DOF; i++)
  {
    pwm.setPWM(ENDINICIAL+(DOF-1-i), 0, map(pose[i], 0, 180, SERVOMIN, SERVOMAX));
    delay(1000);
  }
}

void fechaGarra()
{
  pwm.setPWM(13, 0, map(80, 0, 180, SERVOMIN, SERVOMAX));
}

void abreGarra()
{
  pwm.setPWM(13, 0, map(170, 0, 180, SERVOMIN, SERVOMAX));
}

void pegaFita()
{
  abreGarra();
  int pegar[5] = {160, 45, 90, 90, 130};
  Serial.println("Simbora");
  for(int i=0; i<DOF; i++)
  {
  int pulselength = map(pegar[i], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(ENDINICIAL+i, 0, pulselength);
    delay(1000);
  }
  fechaGarra();
}
