
#include <math.h>

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
volatile unsigned int battery; //Variable para medir el ADC de la bateria

struct smooth{
int readings[NUM_SAMPLES];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
}batt;

#ifdef __cplusplus
extern "C"{
#endif
ISR(ADC_vect){
  if(!OCR1A & !OCR1B){ //Si los motores estan  0,0 - Lee el ADC
  batt.total = batt.total - batt.readings[batt.readIndex];
  // read from the sensor:
  batt.readings[batt.readIndex] = ADC;
  // add the reading to the total:
  batt.total = batt.total + batt.readings[batt.readIndex];
  // advance to the next position in the array:
  batt.readIndex = batt.readIndex + 1;

  // if we're at the end of the array...
  if (batt.readIndex >= NUM_SAMPLES) {
    // ...wrap around to the beginning:
    batt.readIndex = 0;
  }
  // calculate the average:
  battery =  batt.total / NUM_SAMPLES;
  
}
}
#ifdef __cplusplus
} // extern "C"
#endif
void getSpeedsFix(int *speedM1,int *speedM2,int left,int right){

  float D1,D2; //Crea las variables para calcular los duttys
  D1 = mapfloat(left,-400,400,-1.0,1.0); //Duty fijo deseado 
  D2 = mapfloat(right,-400,400,-1.0,1.0); //Dutty fijo deseado 2
  float VrmsFixed1 = float(MIN_VOLT * D1);
  float VrmsFixed2 = float(MIN_VOLT * D2);
  unsigned int current_bat = getBatteryRaw();
  D1 = float(VrmsFixed1 / get_volts(current_bat));
  D2 = float(VrmsFixed2 / get_volts(current_bat));

  *speedM1 = (abs(D1) <= 1.0 ) ? 400*D1 : left; //Calcula los PWM ajustados
  *speedM2 = (abs(D2) <= 1.0 ) ? 400*D2 : right;
}

int16_t getBatteryRaw(){
  //ADCSRA &= ~(1 << ADIE); //Desactiva la interrupción del ADC
  unsigned int val = battery;
  //ADCSRA |=  (1 << ADIE); //Activa la interrupción del ADC
  return val;
}  
void initADC(){
 for (int thisReading = 0; thisReading < NUM_SAMPLES; thisReading++) {
    batt.readings[thisReading] = 0;
  }
  cli();//disable interrupts
  //set up continuous sampling of analog pin 0 at 38.5kHz
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  ADMUX = 0;
  ADMUX |= ((1 << REFS0) | (1 << REFS1)); //set reference voltage
  //ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  ADMUX |= ADC_BATT; 
  sei();//enable interrupts
}
void disableADC(){
  ADCSRA &=  ~(1 << ADIE); //Activa la interrupción del ADC
  ADCSRA &=  ~(1 << ADEN); //enable ADC
  ADCSRA &=  ~(1 << ADATE); //enabble auto trigger

}

