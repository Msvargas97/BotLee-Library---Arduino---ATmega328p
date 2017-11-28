#include "BotLee.h"

//#define SENSOR0  PORTC4
#define SENSOR0  _BV(PORTC3)
#define SENSOR1  _BV(PORTC0)
#define SENSOR2  _BV(PORTB4)
//#define SENSOR4  PORTC5
#define SENSOR3  _BV(PORTC2)
#define SENSOR4  _BV(PORTC1)
#define SENSOR5  _BV(PORTB5)

const uint8_t sensors_mask[NUM_SENSORS] = {SENSOR0,SENSOR1,SENSOR2,SENSOR3,SENSOR4,SENSOR5};

void BotLeeSensors::init(){
  //Activa resistencias pull-up del puerto C
  DDRC &=  0b11000000;
  PORTC |= 0b00111111;
  //Activa resistencias pull-up del puerto B
  DDRB &= (~_BV(5) & ~_BV(4)); 
  PORTB |= (_BV(5) | _BV(4));
}

bool BotLeeSensors::readSensor(uint8_t sensor){
  //init();
  return ((((sensor == 5 || sensor == 2)) ? PINB : PINC) & sensors_mask[sensor]) ? true : false;
}

void BotLeeSensors::readAll(){
  for (uint8_t i = 0; i < NUM_SENSORS; ++i)
        bitWrite(value,i,readSensor(i));
  
}

uint16_t BotLeeSensors::average(){
  bool on_line = 0;
  unsigned long avg=0; // this is for the weighted total, which is long
  // before division
  unsigned int sum=0; // this is for the denominator which is <= 64000
  static uint16_t _lastValue = 0;

  value = 0;
  for (uint8_t i=0;i<  NUM_SENSORS;i++)
  {
    bool state = readSensor(i);
    bitWrite(value,i,state);
    if(state == true) {
      on_line = 1;
    }
    avg += (long)(state) * (i * 1000);
    sum += state;
  }
  if(!on_line)
  {
    // If it last read to the left of center, return 0.
    if(_lastValue < (unsigned int)(NUM_SENSORS-1)*1000/2)
    return 0;
    // If it last read to the right of center, return the max.
    else
    return (NUM_SENSORS-1)*1000;
  }
  _lastValue = avg/sum;
  return _lastValue;
}
