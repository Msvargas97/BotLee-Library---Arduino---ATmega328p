#include "BotLee.h"
#include <stdint.h> //Libreria para tipos de variables
#include <stdlib.h> //Libreria para el manejo de memoria,array,string...
#include <stdarg.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#ifndef pgm_read_byte
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
 #define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
 #define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif


#if !USE_SERIAL_ARDUINO
#define PRINTF_BUF 80 // define the tmp buffer size (change if desired)


static int uart_putchar(char c, FILE *stream);
static FILE uartout = {0} ;
volatile char buffer_rx[BUFFER_SIZE];
volatile bool newDataUART;
void (*_newDataCallback)(void) = NULL;

static int uart_putchar(char c, FILE *stream)
{
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0 ;
}
ISR(USART_RX_vect){
  static size_t i=0;
  static  char dato_rx;
  dato_rx=UDR0;     //Lee el dato recibido
  if(dato_rx == '\n') {
    buffer_rx[i] = '\0';
    i = 0;
    newDataUART = true;
    if ( _newDataCallback != NULL) { //Verifica si se creo la función y la llama
      _newDataCallback();
    }
    }else{
      buffer_rx[i++] = dato_rx;
    }
  } 
void BotLeeSerial::begin(unsigned long baud_rate){
    #define BAUD_PRESCALE1(USART_BAUDRATE) (((F_CPU / (USART_BAUDRATE * 8UL)))-1)
    #define BAUD_PRESCALE2(USART_BAUDRATE) (F_CPU / 8 / USART_BAUDRATE - 1) / 2;
    cli();
    unsigned int ubrr = (baud_rate >= 115200 ) ? BAUD_PRESCALE1(baud_rate) : BAUD_PRESCALE2(baud_rate);
  //
  //Set baud rate 
  UBRR0H = static_cast<unsigned char>(ubrr>>8);
  UBRR0L = static_cast<unsigned char>(ubrr);
  //Enable receiver and transmitter 
  UCSR0B = (1 << RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
 // Set frame format: 8data, 2stop bit 
 if(baud_rate < 115200) UCSR0C = (1<<USBS0)|(3<<UCSZ00);
 else{
  UCSR0C =((0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00));
  UCSR0A =(1<<U2X0);
} 
fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
   // The uart is the standard output device STDOUT.
   stdout = &uartout ;
   sei();
 }
 String BotLeeSerial::getMessage(){
  char tmp[BUFFER_SIZE];
  UCSR0B &= ~(1 << RXCIE0);
  memcpy(tmp,buffer_rx,BUFFER_SIZE);
  String msg = String(tmp);
  memset(buffer_rx,0,BUFFER_SIZE);
  UCSR0B |= (1 << RXCIE0);
  return msg;
}
uint8_t BotLeeSerial::getc(){
  while (!(UCSR0A & _BV(RXC0)));
  return (uint8_t) UDR0;
}
bool BotLeeSerial::isNewData(){
  if(newDataUART){
    newDataUART = false;
    return true;
    }else 
    return false;
  }
  void BotLeeSerial::setNewDataCallback(void (*func)(void)){
    _newDataCallback = func;
  }
void BotLeeSerial::print_char(char c){
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

#ifdef F // check to see if F() macro is available
void BotLeeSerial::print(const __FlashStringHelper *format, ...)
{
 char buf[PRINTF_BUF];
 va_list ap;
 va_start(ap, format);
 #ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)format, ap); // progmem for AVR
  #else
  vsnprintf(buf, sizeof(buf), (const char *)format, ap); // for the rest of the world
  #endif
  for(char *p = &buf[0]; *p; p++) // emulate cooked mode for newlines
     BotLeeSerial::print_char(*p);
  
  va_end(ap);
}
void BotLeeSerial::println(const __FlashStringHelper *format, ...)
{
 char buf[PRINTF_BUF];
 va_list ap;
 va_start(ap, format);
 #ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)format, ap); // progmem for AVR
  #else
  vsnprintf(buf, sizeof(buf), (const char *)format, ap); // for the rest of the world
  #endif
  for(char *p = &buf[0]; *p; p++) // emulate cooked mode for newlines
     BotLeeSerial::print_char(*p);
  BotLeeSerial::print(F("\r\n"));
  va_end(ap);
}
#endif

void BotLeeSerial::printf(const char *format,...){
       va_list arg;
       va_start (arg, format);
       vfprintf(stdout, format, arg);
       va_end (arg);
}
void BotLeeSerial::print(const char *format,...){
       va_list arg;
       va_start (arg, format);
       vfprintf(stdout, format, arg);
       va_end (arg);
}
void BotLeeSerial::println(const char *format,...){
       va_list arg;
       va_start (arg, format);
       vfprintf(stdout, format, arg);
       //vfprintf(stdout,"\r\n",NULL);
       BotLeeSerial::print(F("\r\n"));
       va_end (arg);
}
void BotLeeSerial::print(float fVal)
{
    char result[100];
    int dVal, dec, i;

    fVal += 0.005;   // added after a comment from Matt McNabb, see below.

    dVal = fVal;
    dec = (int)(fVal * 100) % 100;

    memset(result, 0, 100);
    result[0] = (dec % 10) + '0';
    result[1] = (dec / 10) + '0';
    result[2] = '.';

    i = 3;
    while (dVal > 0)
    {
        result[i] = (dVal % 10) + '0';
        dVal /= 10;
        i++;
    }

    for (i=strlen(result)-1; i>=0; i--)
        BotLeeSerial::print_char(result[i]);
}
void BotLeeSerial::println(){
      BotLeeSerial::print(F("\r\n"));
}
#endif