#include <BotLee.h>       //Libreria para el control de motores,sensores..
#include "config.h"
//ARDUINO NANO o Arduino Mini
BotLeeRC decoder;
BotLeeMotors motors;
//BotLeeSerial serial;

int M1, M2;
volatile bool run;
volatile bool flagSelect = false;

void setup() {
  // put your setup code here, to run once:
  initRobot();
  //serial.begin(57600);
}

void loop() {

  rc_control();
  routine_exec();
//  serial.println("%d %d",decoder.readChannelFast(CH_MODE));
//  delay(100);
}
void routine_exec() {
  if (!flagSelect) {
    if (decoder.readChannelFast(CH_SELECT) >= 1900) {
      int mode = decoder.readChannelFast(CH_MODE);
      if (mode >= 1900) {
        slide_left();
      } else if (mode <= 1200) {
        arco_borde();
      } else {
        slide_right();
      }
      flagSelect = true;
    }
  }
}

void rc_control() {
  decoder.getSpeedsRC(&M1, &M2, CH_X, CH_Y); //Obtiene los valores de los canales RC, ya mapeados de -400 a 400
  motors.setSpeeds(M1, M2);
}
void ad_at(){
  getSpeedsFix(&M1,&M2,360,360);
  motors.setSpeeds(-M1,M2);
  delay(150);
  motors.stop();
  delay(15);
  motors.setSpeeds(M1,-M2);
  delay(150);
}
void slide_right() {
  rot90(TURN_RIGHT);
  getSpeedsFix(&M1,&M2,190,360);
  motors.setSpeeds(-M1,M2);
  delay(260);
  rot90(TURN_LEFT);
  motors.stop();
}
void slide_left() {
  rot90(TURN_LEFT);
  getSpeedsFix(&M1,&M2,360,190);
  motors.setSpeeds(-M1,M2);
  delay(260);
  rot90(TURN_RIGHT);
  motors.stop();
}
void arco_borde() {
  rot90(TURN_LEFT);
  arco(TURN_RIGHT);
  rot90(TURN_RIGHT);
}
void initRobot() {
  unsigned long tic = millis();
  motors.stop();  //Detiene motores
  decoder.setNumChannels(MAX_CHANNELS); //Asigna canales
  initADC(); //Inicializa el ADC
  int16_t adc_bat = 0; //Para calcular el promedio de la bateria
  while (millis() - tic <= TIME_START) {
    adc_bat = 0;
    decoder.getSpeedsRC(&M1, &M2, CH_X, CH_Y ); // utilizar | INVERTED  - cuando se desee invertir los valores del canal Ej(min,max) --> (max,min)
    for (uint8_t   i = 0; i < 10; ++i)
    {
      adc_bat += getBatteryRaw();
      delay(1);
    }
  }
  disableADC();
  resetTimer();
  run = true;
}


