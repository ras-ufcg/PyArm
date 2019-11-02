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

#define DEBUG1       true  // if(DEBUG1) Serial.println();
#define DEBUG2       false //
#define SERVOMIN    150   // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX    600   // this is the 'maximum' pulse length count (out of 4096)
#define DOF         5     // número de juntas
#define PASSO       3     // tamanho do incrmento angular em graus a cada interação
#define ENDINICIAL  8     // Endereço da PCA9685 onde foi ligado o motor da base
#define INTEVALO    100   // intervalo de tempo entre as interações em ms

// Os motores devem se ligados em sequência de baixo pra cima a partir da base.

int pose[DOF] =  {90, 90, 90, 90, 90}; // angulos iniciais dos motores {M0, M1, M2, M3, M4}

char terminator = '\n';

int comprimento = 3,
    contadorDeComandos = 0,
    comandoCompleto[DOF],
    incrementos[180/PASSO][DOF]; // [linhas]x[colunas] = [incrementos][motores]

void setup() {
  Serial.begin(9600);
  
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);

  poseInicial();
  fechaGarra();

  if(DEBUG1) Serial.println("Controlador do Braço Pick and Place");
  if(DEBUG1) Serial.println("Status: PRONTO");
}

void loop() {

  if(Serial.available())
  {
    String entrada = Serial.readStringUntil(terminator);

    if(entrada == "00") poseInicial();
    else if(entrada == "fecha") fechaGarra();
    else if(entrada == "abre") abreGarra();
    else if(entrada.toInt()<180) montarComando(entrada.toInt());
  }
}

void montarComando(int angulo)
{
  comandoCompleto[contadorDeComandos++] = angulo;

  if(DEBUG1)Serial.print("Âgulo recebido para o motor M");
  if(DEBUG1)Serial.print(contadorDeComandos-1);
  if(DEBUG1)Serial.print(": ");
  if(DEBUG1)Serial.println(angulo);
  
  if(contadorDeComandos >= DOF)
    {
        contadorDeComandos = 0;
        montarMatrizDeIncrementos();
    }
}

void montarMatrizDeIncrementos()
{
  // O braço está em "pose" e deve ir para "comandoCompleto"
  // Calcular a diferença entre os angulos de cada motor
  int deltaAngulos [DOF];
  for(int i=0; i<DOF; i++)
  {
    deltaAngulos[i] = comandoCompleto[i] - pose[i];
    
    if(DEBUG2) Serial.print("Delta do motor ");
    if(DEBUG2) Serial.print(i);
    if(DEBUG2) Serial.print(": ");
    if(DEBUG2) Serial.println(deltaAngulos[i]);
    
  }

  // Calcular matriz e incrementos
  
  for(int i=0; i<DOF; i++)
  {
    // Calcular quantas linhas de icremento serão nessesárias pra realizar esse movimento
    int n_incrementos = deltaAngulos[i]/PASSO;    
    int resto_inc = deltaAngulos[i]%PASSO;

    if(DEBUG2) Serial.print("Número de incrementos necessários para o motor ");
    if(DEBUG2) Serial.print(i);
    if(DEBUG2) Serial.print(": ");
    if(DEBUG2) Serial.print(n_incrementos);
    if(DEBUG2) Serial.print("\tResto: ");
    if(DEBUG2) Serial.println(resto_inc);
    
    for(int j=0; j<(180/PASSO); j++)
    {      
      incrementos[j][i] = (j<abs(n_incrementos))?  (n_incrementos/abs(n_incrementos))*PASSO : 0;
    }
    
    incrementos[abs(n_incrementos)][i] = resto_inc;
  }
  if(DEBUG2){
    Serial.println("INCREMENTOS:");
    for(int i=0; i<(180/PASSO); i++) 
    {
      Serial.print(i);
      Serial.print(": ");
      for(int j=0; j<DOF; j++)
      { 
        Serial.print("\t");
        Serial.print(incrementos[i][j]);
      }
      Serial.println();
    }
  }
  executaComandos();
}

void executaComandos()
{
  if (DEBUG1)Serial.println("Status: EXECUTANDO COMANDOS");
  for(int i=0; i<(180/PASSO); i++) 
    {
      if(DEBUG2)Serial.print("Iteração ");
      if(DEBUG2)Serial.print(i);
      if(DEBUG2)Serial.println(": ");

      for(int j=0; j<DOF; j++)
      { 
        if(incrementos[i][j]==0)break;
        if(DEBUG2)Serial.print("Posição do motor M");
        if(DEBUG2)Serial.print(j);
        if(DEBUG2)Serial.print(" antes da interação: ");
        if(DEBUG2)Serial.println(pose[j]);
        
        pose[j]+=incrementos[i][j];
        int pulselength = map(pose[j], 0, 180, SERVOMIN, SERVOMAX);
        pwm.setPWM(ENDINICIAL+j, 0, pulselength);

        if(DEBUG2)Serial.print("Posição do motor M");
        if(DEBUG2)Serial.print(j);
        if(DEBUG2)Serial.print(" depois da interação: ");
        if(DEBUG2)Serial.println(pose[j]);
      }
      if(DEBUG2)Serial.println();
      delay(INTEVALO);
    }
  if(DEBUG1){
    Serial.print("POSE FINAL APOS A EXECUÇÃO DOS COMANDOS: ");
    Serial.print("{");
    for(int j=0; j<DOF; j++)
    { 
      Serial.print(pose[j]);
      if(j<DOF-1)Serial.print(",");
    }
    Serial.println("}");
    Serial.println("Status: PRONTO");
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