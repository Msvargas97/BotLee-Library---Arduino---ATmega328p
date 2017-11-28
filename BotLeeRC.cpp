#include "BotLee.h"
#include <math.h>
#include <avr/pgmspace.h> 
#include "SpeedTrig.h"


//Obtiene la mascara según el pin que cambio de estado
#define MASKPCINT0 (1<<2)
#define MASKPCINT1 (1<<3)
#define MASKPCINT2 (1<<4)
#define MASKPCINT3 (1<<5)
#define MASKPCINT4 (1<<6)
#define MASKPCINT5 (1<<7)

//Crea vector global
volatile unsigned long rawIn[MAX_CHANNELS];
volatile uint8_t num_use = MAX_CHANNELS;  
static Speed_Trig SpeedTrig;
//LECTURA DE LOS CANALES RC
ISR(PCINT2_vect)
{
  static uint8_t newbit, oldbit = ~PIND, changed;
  static unsigned long startIn[MAX_CHANNELS];
  static unsigned long time;

  if(num_use > 0){ 
    time = micros();
    newbit = PIND;
    changed = newbit ^ oldbit;
  }
  // This is a new VERY VERY VERY fast method
  // 1 xor operation
  if(num_use > 0)
  if (changed & MASKPCINT0)
  if (newbit & MASKPCINT0) startIn[0] = time;
  else rawIn[0] = time - startIn[0];

  if(num_use > 1)
  if (changed & MASKPCINT1)
  if (newbit & MASKPCINT1) startIn[1] = time;
  else rawIn[1] = time - startIn[1];

  if(num_use > 2)
  if (changed & MASKPCINT2)
  if (newbit & MASKPCINT2) startIn[2] = time;
  else rawIn[2] = time - startIn[2];

  if(num_use > 3)
  if (changed & MASKPCINT3)
  if (newbit & MASKPCINT3) startIn[3] = time;
  else rawIn[3] = time - startIn[3];

  if(num_use > 4)
  if (changed & MASKPCINT4)
  if (newbit & MASKPCINT4) startIn[4] = time;
  else rawIn[4] = time - startIn[4];

  if(num_use > 5)
  if (changed & MASKPCINT5)
  if (newbit & MASKPCINT5) startIn[5] = time;
  else rawIn[5] = time - startIn[5];

  oldbit = newbit;
}

int16_t BotLeeRC::readChannel(unsigned char ch,int16_t min,int16_t max){
//BotLeeRC::init(); //Inicializa si es necesario
if(min == 0 && max == 0){
  BotLeeRC::movil_mean(ch); //Calcula el promedio movil 
  return  average[ch];
  }else {
    return BotLeeRC::mapChannel(ch,min,max);
  }
}
int16_t BotLeeRC::readChannelFast(unsigned char ch){
  uint8_t SREG_old = SREG;
  cli();
  int raw = (rawIn[ch] > 900 && rawIn[ch] < 2100) ? rawIn[ch] : 0;
  SREG = SREG_old;
  return raw;
}
int16_t BotLeeRC::mapChannel(unsigned char ch,int16_t min,int16_t max){
  BotLeeRC::movil_mean(ch); //Calcula el promedio movil 
  if(average[ch] != 0){    
    if (average[ch] > CENTER){
      value[ch] = map(average[ch], CENTER + 1, MAX_VAL,0, max);
      } else if (average[ch] < 1500) {
        value[ch] = map(average[ch], 1499, MIN_VAL,0,min);
      }
      value[ch] = constrain(value[ch], min, max);
      return value[ch];
      }else{
        return 0;
      }
}

 void BotLeeRC::getSpeedsRC(int16_t *speedM1,int16_t *speedM2,int16_t ch_x,int16_t ch_y,bool mode,bool fast,bool mixer){
    //Crea un puntero tipo registro para acceder de manera rapida
    register int16_t *M1 = speedM1;
    register int16_t *M2 = speedM2;

    float x = 0.0, y = 0.0, left=0.0, right=0.0;
    if(!mode){
      //Lee los canales, solo los 3 primeros bits
      x = BotLeeRC::readChannel(ch_x & 0b111, -400, 400);
      y = BotLeeRC::readChannel(ch_y & 0b111, -400, 400);
      if(bitRead(ch_x,7)){
        x *= -1.0;
      }
      if(bitRead(ch_y,7)){
        y *= -1.0;
      }
      }else{
        x = ch_x;
        y = ch_y;
      }

      y /= 400;
      x /= 400;
      steering(&left, &right, x, y,fast,mixer);
      *M1 = SpeedTrig.floatToInt(400 * left);
      *M2 = SpeedTrig.floatToInt(400 * right);
      if (*M1 >= 390) *M1 = 400;
      else if (*M1 <= -390) *M1 = -400;
      if (*M2 >= 390) *M2 = 400;
      else if (*M2 <= -390) *M2 = -400;
 }

void BotLeeRC::steering(float *l1, float *r1, float x, float y,bool fast,bool mixer) {
  //Crea un puntero tipo registro para acceder de manera rapida
  register float *left = l1;
  register float *right = r1;

  float r, theta;
  //Convertir a coordenadas polares
  r = hypot(x,y); /* better than sqrt(x*x+y*y) */
  if(!fast)
    theta = atan2(y,x);
  else
    theta = SpeedTrig.atan2(y, x);
  //Rotar 45 grados
  if(mixer)
  theta = theta - M_PI / 4;
  
  //Regresa a cartesianas
  if(!fast){
    *left = r * cos(theta);
    *right = r * sin(theta);
    }else{
      *left  = r * SpeedTrig.cos(radiansToDegrees(theta));
      *right = r * SpeedTrig.sin(radiansToDegrees(theta));
  }

  // Reescala las nuevas coordenadas
  *left = (*left) * 1.4142135623730950488016887242097f;
  *right = (*right) * 1.4142135623730950488016887242097f;
  // Restringir de -1/+1
  *left = max(-1.0, min(*left, 1.0));
  *right = max(-1.0, min(*right, 1.0));
}


void BotLeeRC::setNumChannels(uint8_t num){
  num_use = num;
  BotLeeRC::init(); 
}

void BotLeeRC::movil_mean(uint8_t ch){
  total[ch] = total[ch] - readings[readIndex[ch]][ch];
  uint8_t SREG_old = SREG;
  cli();
  readings[readIndex[ch]][ch] = (rawIn[ch] > 900 && rawIn[ch] < 2100) ? rawIn[ch] : 0;
  SREG = SREG_old;
  total[ch] = total[ch] + readings[readIndex[ch]][ch];
  readIndex[ch] = readIndex[ch] + 1;
  if (readIndex[ch] >= NUM_SAMPLES) {
    readIndex[ch] = 0;
  }
  average[ch] = total[ch] / NUM_SAMPLES;
  if (average[ch] >= 1500 && average[ch] <= 1520) {
    average[ch] = 1500;
    value[ch] = 0;
  }
  else if (average[ch] >= 1095 && average[ch] <= 1110) {
    average[ch] = 1095;
  }
  else if (average[ch] >= 1910 && average[ch] <= 1920) {
    average[ch] = 1920;
    } else if (abs(average[ch] - last_average[ch]) < 10) {
      average[ch] = last_average[ch];
    }

    last_average[ch] = average[ch];
}

void BotLeeRC::init2(){
    cli();
  DDRD = DDRD & 0b00000011; //Setea los pines como entradas
  PORTD = 0b11111100; //Habilita las resistencias Pull-Up
  if(num_use > 0)
    PCICR |= (1 << PCIE2); //Habilita la interrupcion por cambio de estado
    if(num_use > 1)
    PCMSK2 |= (1 << PCINT18); // Pin D2
    if(num_use > 2)
    PCMSK2 |= (1 << PCINT19); //  Pin D3
    if(num_use > 3)
    PCMSK2 |= (1 << PCINT20); //  Pin D4
    if(num_use > 3)
    PCMSK2 |= (1 << PCINT21); //  Pin D5
    if(num_use > 4)
    PCMSK2 |= (1 << PCINT22); //  Pin D6
    if(num_use > 5)
    PCMSK2 |= (1 << PCINT23); //  Pin D7
    PCIFR = 0xFF;
  sei(); // enable interrupts
}
