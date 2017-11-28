#include "BotLee.h"

//#####################  BotLee Motors ####################################
void BotLeeMotors::init2(){
  cli();//disable interrupts
    DDRB |= _BV(1) | _BV(2) |  _BV(0) | _BV(3); //Setea los pines como salidas
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;//Configura el PWM a 20KHz
    OCR1A = 0;
    OCR1B = 0;
    sei();//enable interrupts
  }


  void BotLeeMotors::setSpeeds(int16_t speedM1,int16_t speedM2){
    //static bool flag1,flag2;
    static int lastSpeedM1,lastSpeedM2;
    //if(lastSpeedM1 <= 0 && speedM1 > 0) flag1 = true;

    if(lastSpeedM1 <= 0 && speedM1 > 0){
      BotLeeMotors::setSpeedM1(-10);
      delayMicroseconds(200);
      lastSpeedM1 = speedM1;
    }
    BotLeeMotors::setSpeedM1(speedM1);
    if(lastSpeedM2 <= 0 && speedM2 > 0){
      BotLeeMotors::setSpeedM2(-10);
      delayMicroseconds(200);
      lastSpeedM2 = speedM2;
    }
    BotLeeMotors::setSpeedM2(speedM2);
  }

  void BotLeeMotors::setSpeedM1(int16_t speed){
    BotLeeMotors::init(); 
    bool reverse = 0;
    if (speed < 0)
    {
        speed = -speed; //Establece que la velocidad sea positiva
        reverse = 1;     //Guarda la dirección
      }
      if (speed > 400)    //MAXIMO velocidad
      {
        speed = 400;
      }
      if (reverse) PORTB |= _BV(0);
      else  PORTB &= ~_BV(0);
      OCR1A = speed;
    }

    void BotLeeMotors::setSpeedM2(int16_t speed){
      BotLeeMotors::init();
      bool reverse = 0;
      if (speed < 0)
      {
        speed = -speed; //Establece que la velocidad sea positiva
        reverse = 1;    //Guarda la dirección
      }
      if (speed > 400)    //MAXIMA velocidad
      {
        speed = 400;
      }
      if (reverse) PORTB |= _BV(3);
      else  PORTB &= ~_BV(3);
      OCR1B = speed;
    }

    void BotLeeMotors::stop(){
      BotLeeMotors::setSpeeds(0,0);
    }
    bool BotLeeMotors::isStop(){
     return (!OCR1A & !OCR1B);
   }