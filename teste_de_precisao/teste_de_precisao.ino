/*  
 *  
 *  Autor: Lyang Leme de Medeiros - 03/02/2019
 *  Target: Arduino UNO
 * 
 *  Datasheets:
 *    ATmega328P - https://bitbucket.org/lyangmedeiros/vera/src/master/doc/datasheets/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 *    KC8801 - https://bitbucket.org/lyangmedeiros/vera/src/master/doc/datasheets/201403091840435763.zh-CN.en.pdf
 * 
 *  Fontes:
 *    GreatScott! - https://www.youtube.com/watch?v=IdL0_ZJ7V2s
 *    WRKits - https://www.youtube.com/watch?v=wXGAbordDqs
 *    mrfid72 - https://www.youtube.com/watch?v=x425HHUWhF8
 * 
 *  Observações: 
 * 
 */


// Mapeamento de Hardware
#define   pwm_out   9   // Sinal de PWM - FIO LARANJA DO SERVO
#define   bot1      2   // Push Button ligado ao GND
#define   bot2      4   // Push Button ligado ao GND

// Constantes
#define   periodo   0.02  // 20 ms

// Variáveis Globais
float lInf = 0.5;        // Limite inferior
float lSup = 2.6;        // Limite Superior
float duty = 0.5;        // Duty cycle inicial

bool bt1 = 0;            // Flag para o estado do botão 1
bool bt2 = 0;            // Flag para o estado do botão 2

void setup() 
{
  
  pinMode(pwm_out, OUTPUT);
  pinMode(bot1, INPUT_PULLUP);
  pinMode(bot2, INPUT_PULLUP);

/*                                TCCRIA
.---------.--------.--------.--------.--------.--------.--------.-------.-------.
| Bit     | 7      | 6      | 5      | 4      | 3      | 2      | 1     | 0     |
:---------+--------+--------+--------+--------+--------+--------+-------+-------:
| x80     | COM1A1 | COM1A0 | COM1B1 | COM1B0 | -      | -      | WGM11 | WGM10 |
:---------+--------+--------+--------+--------+--------+--------+-------+-------:
| r/w     | R/W    | R/W    | R/W    | R/W    | R      | R      | R/W   | R/W   |
:---------+--------+--------+--------+--------+--------+--------+-------+-------:
| initial | 0      | 0      | 0      | 0      | 0      | 0      | 0     | 0     |
'---------'--------'--------'--------'--------'--------'--------'-------'-------'

                                TCCR1B
.---------.-------.-------.-------.-------.-------.------.------.------.
| Bit     | 7     | 6     | 5     | 4     | 3     | 2    | 1    | 0    |
:---------+-------+-------+-------+-------+-------+------+------+------:
| x81     | ICNC1 | ICES1 | -     | WGM13 | WGM12 | CS12 | CS11 | CS10 |
:---------+-------+-------+-------+-------+-------+------+------+------:
| r/w     | R/W   | R/W   | R     | R/W   | R/W   | R/W  | R/W  | R/W  |
:---------+-------+-------+-------+-------+-------+------+------+------:
| initial | 0     | 0     | 0     | 0     | 0     | 0    | 0    | 0    |
'---------'-------'-------'-------'-------'-------'------'------'------'
 
 */

  TCCR1A = 0b10000010;
  TCCR1B = 0b00011010;
  
  // WGM13  WGM12  WGM11  WGM10    
  //   1      1      1      0
  // Configura o TCNT1 de acordo com o modo 14 da tabela 
  // 15.5. Waveform Generation Mode Bit Description (ver datasheetATmega328P)
  
  // Compare Output mode (COM1x1:0)
  // COM1A1: 1, COM1A0: 0
  // COM1B1: 0, COM1B0: 0
  // Com essa configuração, limpa o OC1A e configura para BOTTOM (contar a partir do valor mínimo)
  // O OC1A é também uma saída no modo PWM do timer.

  // CS12 CS11 CS10
  //  0     1   0
  // seta o prescaler para 8 - DIVIDE O CLOCK POR 8
  // 16 MHz/8 = 2 MHz

  // Input Capture Register (ICR)
  // No WGM 14, o TOP do contador é definido pelo valor
  // armazenado no seu ICR. 
  ICR1 = 2.0E6 * periodo; // = 2000000*0.02 = 40.0e3
 
  // Timer interrupt mask register
  TIMSK0 = 0; // Desliga a máscara de interrupção do TIMER0
  // Aparentemente esse registrador, por padrão é ligado e atrapalha
  // o PWM gerado pelo TIMER1. É necessário um estudo aprofundado sobre
  // essa interferência e se ao desligar esse TIMSK não estamos afetando
  // outras funções do microcontrolador.
  
}// fim do setup

void loop() 
{
  
   // Correção de limites    
   //if (duty < lInf) duty = lInf;
   if (duty > lSup) duty = lSup;
    
   // Output compare register, ver pg 101
   OCR1A = duty * 2000; 

   
   if(digitalRead(bot1) == LOW) debounce1();
   else bt1 = 0;
   
   if(digitalRead(bot2) == LOW) debounce2();
   else bt2 = 0; 


}

void debounce1()
{
   if(!bt1) duty = 0.6;
   
}

void debounce2()
{
  if(!bt2) duty = 0.2;
}
