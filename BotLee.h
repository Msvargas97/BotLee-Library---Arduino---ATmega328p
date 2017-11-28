#ifndef USE_SERIAL_ARDUINO
#define USE_SERIAL_ARDUINO false
#endif

#if !USE_SERIAL_ARDUINO
#define HardwareSerial_h //Deshabilita la libreria Serial
#endif

#include <avr/io.h>
#include <stdint.h> //Libreria para tipos de variables
#include <stdlib.h> //Libreria para el manejo de memoria,array,string...
#include <stdarg.h> //Libreria para añadir argumentos a las funciones
#include <stdio.h> //Libreria para usar y configurar printf
#include <util/twi.h>
#include <Adafruit_GFX.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "Arduino.h"


// //CONSTANTES PARA LOS CANALES RC
#define MAX_CHANNELS 6
#define MIN_TIME 	 600	  //Filtro en microsegundos
#define NUM_SAMPLES	 10	  //Número de muestras, promedio movil
#define CENTER 		 1520   //Centro en us
#define MIN_VAL	1095
#define MAX_VAL	1920
#define INVERTED	(1<<7)
#define NO_READ_CH  true
#define READ_CH  false

//CONSTANTES PARA LOS SENSORES
#define DIGITAL	0
#define ANALOG	1
#define DISABLE	2
#define NUM_SENSORS 6
#define BUFFER_SIZE 64



#define WIDTH 128
#define HEIGHT  64
#define SSD1306_LCDHEIGHT HEIGHT
#define SSD1306_LCDWIDTH  WIDTH
#define BLACK 0
#define WHITE 1
#define INVERSE 2
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A
#define I2C_ADDRESS   0x3C  // 011110+SA0+RW - 0x3C or 0x3D
#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }

#define F_SCL 400000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)
#define I2C_READ 0x01
#define I2C_WRITE 0x00

/*
class BotLeeIR : public IRrecv {

};*/
#if !USE_SERIAL_ARDUINO
class BotLeeSerial {
public:
  BotLeeSerial(){

  }
  void begin(unsigned long baud_rate);
  uint8_t getc();
  void printf(const char *format,...);
  void print(const char *format,...);
  void println(const char *format,...);
  void println(const __FlashStringHelper *format, ...);
  void print(const __FlashStringHelper *format, ...);
  void print_char(char c);
  void println();
  void print(float fVal);
  String getMessage();
  void setNewDataCallback(void (*func)(void));
  bool isNewData();
  ~BotLeeSerial(){
  }

};
#endif

class BotLeeRC{
public:
	BotLeeRC(){

	}
	int16_t readChannel(unsigned char ch,int16_t min=0,int16_t max=0);
	int16_t readChannelFast(unsigned char ch);
  void setNumChannels(uint8_t num);
	void getSpeedsRC(int16_t *left,int16_t *right,int16_t ch_x,int16_t ch_y,bool mode=false,bool fast = true,bool mixer = true);
	int  mapChannel(unsigned char ch,int16_t min,int16_t max);
  void steering(float *left, float *right, float x, float y,bool fast,bool mixer);
  ~BotLeeRC(){

	}
private:	
	void movil_mean(uint8_t ch);
	
  int readings[NUM_SAMPLES][MAX_CHANNELS];      // the readings from the analog input
	int readIndex[MAX_CHANNELS];              // the index of the current reading
	int total[MAX_CHANNELS] ;                  // the running total
	int average[MAX_CHANNELS], last_average[MAX_CHANNELS];
	int16_t  value[MAX_CHANNELS];
	static inline void init()
	{

		static unsigned char initialized = 0;

		if (!initialized)
		{
			initialized = 1;
			init2();
		}
	}
	static void init2();
};

class BotLeeSensors{
public:
	BotLeeSensors(){

	}
  void init();
  void readAll();
	bool readSensor(uint8_t sensor);
  uint16_t average();
  uint8_t value;
	~BotLeeSensors(){
	}
private:
  
};

class BotLeeMotors {
public:
 BotLeeMotors(){

 }
 static void setSpeeds(int16_t speedM1, int16_t speedM2);
 static void setSpeedM1(int16_t speed);
 static void setSpeedM2(int16_t speed);
 static void stop();
 bool isStop();
 ~BotLeeMotors(){
 
 }
private:
	static inline void init()
	{
		static unsigned char initialized = 0;

		if (!initialized)
		{
			initialized = 1;
			init2();
		}
	}
	// initializes timers 0 and 2 for proper PWM generation
	static void init2();
};


static uint8_t *buffer;
class BotLeeOLED : public Adafruit_GFX {
public:
  BotLeeOLED() : Adafruit_GFX(WIDTH, HEIGHT) {
  }
  void begin() {
    _vccstate = SSD1306_SWITCHCAPVCC;
    _i2caddr = (I2C_ADDRESS << 1) | I2C_WRITE;
      i2c_init(); //inicializar i2c
      ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
      ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
      ssd1306_command(0x80);                                  // the suggested ratio 0x80

      ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
      ssd1306_command(SSD1306_LCDHEIGHT - 1);

      ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
      ssd1306_command(0x0);                                   // no offset
      ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
      ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
      if (_vccstate == SSD1306_EXTERNALVCC)
      {
        ssd1306_command(0x10);
      }
      else
      {
        ssd1306_command(0x14);
      }
      ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
      ssd1306_command(0x00);                                  // 0x0 act like ks0108
      ssd1306_command(SSD1306_SEGREMAP | 0x1);
      ssd1306_command(SSD1306_COMSCANDEC);
      ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
      ssd1306_command(0x12);
      ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
      if (_vccstate == SSD1306_EXTERNALVCC)
      {
        ssd1306_command(0x9F);
      }
      else
      {
        ssd1306_command(0xCF);
      }
      ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
      if (_vccstate == SSD1306_EXTERNALVCC)
      {
        ssd1306_command(0x22);
      }
      else
      {
        ssd1306_command(0xF1);
      }
      ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
      ssd1306_command(0x40);
      ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
      ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

      ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

      ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
    }
    void ssd1306_command(uint8_t c) {
      i2c_start(_i2caddr);
      if (i2c_write(0x00)) return ;
      if (i2c_write(c)) return ;
      i2c_stop();
    }
    void displayOn(){
      ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
    }
    void displayOff(){
      ssd1306_command(SSD1306_DISPLAYOFF);//--turn on oled panel
    }
    void initBuffer(size_t size = WIDTH * HEIGHT / 8) {
      buffer = (unsigned char*) malloc(size);
      clearDisplay();
    }
    void freeBuffer() {
      free(buffer);
    }
    void drawPixel(int16_t x, int16_t y, uint16_t color) {
      if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()) || buffer == NULL)
        return;

      // check rotation, move pixel around if necessary
      switch (getRotation()) {
        case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
        case 2:
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        break;
        case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
      }
      // x is which column
      switch (color)
      {
        case WHITE:   buffer[x + (y / 8)*SSD1306_LCDWIDTH] |=  (1 << (y & 7)); break;
        case BLACK:   buffer[x + (y / 8)*SSD1306_LCDWIDTH] &= ~(1 << (y & 7)); break;
        case INVERSE: buffer[x + (y / 8)*SSD1306_LCDWIDTH] ^=  (1 << (y & 7)); break;
      }

    }

    void show(void) {
      ssd1306_command(SSD1306_COLUMNADDR);
      ssd1306_command(0);   // Column start address (0 = reset)
      ssd1306_command(SSD1306_LCDWIDTH - 1); // Column end address (127 = reset)

      ssd1306_command(SSD1306_PAGEADDR);
      ssd1306_command(0); // Page start address (0 = reset)
      ssd1306_command(7); // Page end address

      // I2C
      for (uint16_t i = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8); i++) {
        // send a bunch of data in one xmission
        i2c_start(_i2caddr);
        i2c_write(0x40);
        for (uint8_t x = 0; x < 16; x++) {
          i2c_write(buffer[i]);
          i++;
        }
        i--;
        i2c_stop();
      }
    }
    void clearDisplay(void) {
      if (buffer != NULL)memset(buffer, 0, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));
    }

    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
      boolean bSwap = false;
      if(buffer == NULL) return;
      switch (rotation) {
        case 0:
          // 0 degree rotation, do nothing
        break;
        case 1:
          // 90 degree rotation, swap x & y for rotation, then invert x
        bSwap = true;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
        case 2:
          // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        x -= (w - 1);
        break;
        case 3:
          // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
        bSwap = true;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        y -= (w - 1);
        break;
      }

      if (bSwap) {
        drawFastVLineInternal(x, y, w, color);
      } else {
        drawFastHLineInternal(x, y, w, color);
      }
    }

    void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) {
      // Do bounds/limit checks
      if (y < 0 || y >= HEIGHT) {
        return;
      }

      // make sure we don't try to draw below 0
      if (x < 0) {
        w += x;
        x = 0;
      }

      // make sure we don't go off the edge of the display
      if ( (x + w) > WIDTH) {
        w = (WIDTH - x);
      }

      // if our width is now negative, punt
      if (w <= 0) {
        return;
      }

      // set up the pointer for  movement through the buffer
      register uint8_t *pBuf = buffer;
      // adjust the buffer pointer for the current row
      pBuf += ((y / 8) * SSD1306_LCDWIDTH);
      // and offset x columns in
      pBuf += x;

      register uint8_t mask = 1 << (y & 7);

      switch (color)
      {
        case WHITE:         while (w--) {
          *pBuf++ |= mask;
        }; break;
        case BLACK: mask = ~mask;   while (w--) {
          *pBuf++ &= mask;
        }; break;
        case INVERSE:         while (w--) {
          *pBuf++ ^= mask;
        }; break;
      }
    }

    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
      bool bSwap = false;
      switch (rotation) {
        case 0:
        break;
        case 1:
          // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
        bSwap = true;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        x -= (h - 1);
        break;
        case 2:
          // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        y -= (h - 1);
        break;
        case 3:
          // 270 degree rotation, swap x & y for rotation, then invert y
        bSwap = true;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
      }

      if (bSwap) {
        drawFastHLineInternal(x, y, h, color);
      } else {
        drawFastVLineInternal(x, y, h, color);
      }
    }


    void drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {

      // do nothing if we're off the left or right side of the screen
      if (x < 0 || x >= WIDTH) {
        return;
      }

      // make sure we don't try to draw below 0
      if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

      }

      // make sure we don't go past the height of the display
      if ( (__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
      }

      // if our height is now negative, punt
      if (__h <= 0) {
        return;
      }

      // this display doesn't need ints for coordinates, use local byte registers for faster juggling
      register uint8_t y = __y;
      register uint8_t h = __h;


      // set up the pointer for fast movement through the buffer
      register uint8_t *pBuf = buffer;
      // adjust the buffer pointer for the current row
      pBuf += ((y / 8) * SSD1306_LCDWIDTH);
      // and offset x columns in
      pBuf += x;

      // do the first partial byte, if necessary - this requires some masking
      register uint8_t mod = (y & 7);
      if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if ( h < mod) {
          mask &= (0XFF >> (mod - h));
        }

        switch (color)
        {
          case WHITE:   *pBuf |=  mask;  break;
          case BLACK:   *pBuf &= ~mask;  break;
          case INVERSE: *pBuf ^=  mask;  break;
        }

        // fast exit if we're done here!
        if (h < mod) {
          return;
        }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
      }


      // write solid bytes while we can - effectively doing 8 rows at a time
      if (h >= 8) {
        if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
          do  {
            *pBuf = ~(*pBuf);

            // adjust the buffer forward 8 rows worth of data
            pBuf += SSD1306_LCDWIDTH;

            // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
            h -= 8;
          } while (h >= 8);
        }
        else {
          // store a local value to work with
          register uint8_t val = (color == WHITE) ? 255 : 0;

          do  {
            // write our value in
            *pBuf = val;

            // adjust the buffer forward 8 rows worth of data
            pBuf += SSD1306_LCDWIDTH;

            // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
            h -= 8;
          } while (h >= 8);
        }
      }

      // now do the final partial byte, if necessary
      if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color)
        {
          case WHITE:   *pBuf |=  mask;  break;
          case BLACK:   *pBuf &= ~mask;  break;
          case INVERSE: *pBuf ^=  mask;  break;
        }
      }
    }

    // startscrollright
    // Activate a right handed scroll for rows start through stop
    // Hint, the display is 16 rows tall. To scroll the whole display, run:
    // display.scrollright(0x00, 0x0F)
    void startscrollright(uint8_t start, uint8_t stop) {
      ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
      ssd1306_command(0X00);
      ssd1306_command(start);
      ssd1306_command(0X00);
      ssd1306_command(stop);
      ssd1306_command(0X00);
      ssd1306_command(0XFF);
      ssd1306_command(SSD1306_ACTIVATE_SCROLL);
    }

    // startscrollleft
    // Activate a right handed scroll for rows start through stop
    // Hint, the display is 16 rows tall. To scroll the whole display, run:
    // display.scrollright(0x00, 0x0F)
    void startscrollleft(uint8_t start, uint8_t stop) {
      ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
      ssd1306_command(0X00);
      ssd1306_command(start);
      ssd1306_command(0X00);
      ssd1306_command(stop);
      ssd1306_command(0X00);
      ssd1306_command(0XFF);
      ssd1306_command(SSD1306_ACTIVATE_SCROLL);
    }

    // startscrolldiagright
    // Activate a diagonal scroll for rows start through stop
    // Hint, the display is 16 rows tall. To scroll the whole display, run:
    // display.scrollright(0x00, 0x0F)
    void startscrolldiagright(uint8_t start, uint8_t stop) {
      ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
      ssd1306_command(0X00);
      ssd1306_command(SSD1306_LCDHEIGHT);
      ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
      ssd1306_command(0X00);
      ssd1306_command(start);
      ssd1306_command(0X00);
      ssd1306_command(stop);
      ssd1306_command(0X01);
      ssd1306_command(SSD1306_ACTIVATE_SCROLL);
    }

    // startscrolldiagleft
    // Activate a diagonal scroll for rows start through stop
    // Hint, the display is 16 rows tall. To scroll the whole display, run:
    // display.scrollright(0x00, 0x0F)
    void startscrolldiagleft(uint8_t start, uint8_t stop) {
      ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
      ssd1306_command(0X00);
      ssd1306_command(SSD1306_LCDHEIGHT);
      ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
      ssd1306_command(0X00);
      ssd1306_command(start);
      ssd1306_command(0X00);
      ssd1306_command(stop);
      ssd1306_command(0X01);
      ssd1306_command(SSD1306_ACTIVATE_SCROLL);
    }

    void stopscroll(void) {
      ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
    }

    // Dim the display
    // dim = true: display is dimmed
    // dim = false: display is normal
    void dim(boolean dim) {
      uint8_t contrast;

      if (dim) {
        contrast = 0; // Dimmed display
      } else {
        if (_vccstate == SSD1306_EXTERNALVCC) {
          contrast = 0x9F;
        } else {
          contrast = 0xCF;
        }
      }
      // the range of contrast to too small to be really useful
      // it is useful to dim the display
      ssd1306_command(SSD1306_SETCONTRAST);
      ssd1306_command(contrast);
    }
  private:
    uint8_t _i2caddr, _vccstate;
    void i2c_init(void)
    {
      TWBR = (uint8_t)TWBR_val;
    }
    uint8_t i2c_start(uint8_t address)
    {
      // reset TWI control register
      TWCR = 0;
      // transmit START condition
      TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
      // wait for end of transmission
      while ( !(TWCR & (1 << TWINT)) );

      // check if the start condition was successfully transmitted
      if ((TWSR & 0xF8) != TW_START) {
        return 1;
      }

      // load slave address into data register
      TWDR = address;
      // start transmission of address
      TWCR = (1 << TWINT) | (1 << TWEN);
      // wait for end of transmission
      while ( !(TWCR & (1 << TWINT)) );

      // check if the device has acknowledged the READ / WRITE mode
      uint8_t twst = TW_STATUS & 0xF8;
      if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

      return 0;
    }

    uint8_t i2c_write(uint8_t data)
    {
      // load data into data register
      TWDR = data;
      // start transmission of data
      TWCR = (1 << TWINT) | (1 << TWEN);
      // wait for end of transmission
      while ( !(TWCR & (1 << TWINT)) );

      if ( (TWSR & 0xF8) != TW_MT_DATA_ACK ) {
        return 1;
      }

      return 0;
    }
    void i2c_stop(void)
    {
      // transmit STOP condition
      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    }


  };
