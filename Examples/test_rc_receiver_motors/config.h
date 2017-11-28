#define TIME_START  3000
#define CH_X 0
#define CH_Y 1 | INVERTED
#define CH_MODE  5 //Selecciona el modo
#define CH_EN_PISO  3 //Habilita o deshabilita los sensores de piso
#define CH_SELECT 2 //Cambia
#define BAUD_RATE 57600 //Velocidad de comunicación serial
#define TIME_START  3000  //Tiempo de espera antes de iniciar
#define SENSOR_L  1
#define SENSOR_R  2
#define PISO_L    0
#define PISO_R    3
#define NUM_CELLS_BATT  8 //Número de celdas a usar
#define MIN_VOLT  (3.7*NUM_CELLS_BATT) //Voltaje minimo
#define MAX_VOLT  (4.2*NUM_CELLS_BATT) //Voltaje maximo
#define ADC_BATT  7 //Pin al que esta conectado el medidor de bateria
#define R1 119 //Resistencia R1 del divisor de tension
#define R2 3.29 //Resistencia R2 del divisor 
#define OFFSET_V  0 //Offset para calcular el valor correcto del a bateria
#define get_volts(x) ((float)(x*(1.1/1023)*((R1+R2)/R2)) - OFFSET_V)
#define NUM_SAMPLES 10  //Número de muestras para el promedio movil bateria
//Funciones timer
#define TIMER1_ON() TIMSK1 |= (1 << TOIE1) //Activa el timer1 20KHz --> 20 ticks == 1ms
#define TIMER1_OFF()  TIMSK1 &= ~(1 << TOIE1) //Desactiva el timer1

#define TURN_LEFT -1 //Girar a la izquierda
#define TURN_RIGHT   1 //Girar a la derecha
