/*
   IR_RC5.h

   Created: 30/03/2017 10:09:27 p. m.
    Author: Michael Vargas
*/


#ifndef IR_RC5_H_
#define IR_RC5_H_

#include <avr/interrupt.h>

#define USECPERTICK 50  // microseconds per clock interrupt tick
#define RAWBUF 100 // Length of raw duration buffer
#define ERR 0
#define DECODED 1
// Marks tend to be 100us too long, and spaces 100us too short
// when received due to sensor lag.
#define MARK_EXCESS 100
#ifdef F_CPU
#define SYSCLOCK F_CPU     // main Arduino clock
#else
#define SYSCLOCK 16000000  // main Arduino clock
#endif
#define PANASONIC_HDR_MARK 3502
#define PANASONIC_HDR_SPACE 1750
#define PANASONIC_BIT_MARK 502
#define PANASONIC_ONE_SPACE 1244
#define PANASONIC_ZERO_SPACE 400
#define MIN_RC5_SAMPLES 11
#define RC5_T1		889
#define RC5_RPT_LENGTH	46000
#define SHARP_BITS 15
#define DISH_BITS 16
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261u
#define UNKNOWN 2
#define TOLERANCE 25  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.)
#define UTOL (1.0 + TOLERANCE/100.)

#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)

#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))
// receiver states
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5

#define TIMER_RESET
#define TIMER_ENABLE_PWM     (TCCR2A |= _BV(COM2B1))
#define TIMER_DISABLE_PWM    (TCCR2A &= ~(_BV(COM2B1)))
#define TIMER_ENABLE_INTR    (TIMSK2 = _BV(OCIE2A))
#define TIMER_DISABLE_INTR   (TIMSK2 = 0)
#define TIMER_INTR_NAME      TIMER2_COMPA_vect
#define TIMER_CONFIG_KHZ(val) ({ \
    const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
    TCCR2A = _BV(WGM20); \
    TCCR2B = _BV(WGM22) | _BV(CS20); \
    OCR2A = pwmval; \
    OCR2B = pwmval / 3; \
  })
#define TIMER_COUNT_TOP      (SYSCLOCK * USECPERTICK / 1000000)
#if (TIMER_COUNT_TOP < 256)
#define TIMER_CONFIG_NORMAL() ({ \
    TCCR2A = _BV(WGM21); \
    TCCR2B = _BV(CS20); \
    OCR2A = TIMER_COUNT_TOP; \
    TCNT2 = 0; \
  })
#else
#define TIMER_CONFIG_NORMAL() ({ \
    TCCR2A = _BV(WGM21); \
    TCCR2B = _BV(CS21); \
    OCR2A = TIMER_COUNT_TOP / 8; \
    TCNT2 = 0; \
  })
#endif
// IR detector output is active low
#define MARK  0
#define SPACE 1

#define TOPBIT 0x80000000
// Decoded value for NEC when a repeat code is received
#define REPEAT 0xffffffff
int MATCH(int measured, int desired) {
  return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);
}
int MATCH_MARK(int measured_ticks, int desired_us) {
  return MATCH(measured_ticks, (desired_us + MARK_EXCESS));
}
int MATCH_SPACE(int measured_ticks, int desired_us) {
  return MATCH(measured_ticks, (desired_us - MARK_EXCESS));
}
// information for the interrupt handler
typedef struct {
  uint8_t rcvstate;          // state machine
  uint8_t blinkflag;         // TRUE to enable blinking of pin 13 on IR processing
  unsigned int timer;        // state timer, counts 50uS ticks.
  unsigned int rawbuf[RAWBUF]; // raw data
  uint8_t rawlen;            // counter of entries in rawbuf
} irparams_t;

volatile irparams_t irparams;
class decode_results {
  public:
    unsigned long value; // Decoded value
    volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
    int rawlen; // Number of records in rawbuf.
};

class IRrecv
{
  public:
    IRrecv() {
      /*DDRB &= ~_BV(PORTB0);
        PORTB |= _BV(PORTB0);
        PORTD &= ~_BV(PORTD3);
        DDRD |= _BV(PORTD3);*/
    };
    void setStateLed(bool state) {
      irparams.blinkflag = state;
    }
    int decode(decode_results *results) {
      results->rawbuf = irparams.rawbuf;
      results->rawlen = irparams.rawlen;
      if (irparams.rcvstate != STATE_STOP) {
        return ERR;
      }
      if (decodeRC5(results)) {
        return DECODED;
      }
      //if (decodeHash(results)) {
      //	return DECODED;
      //}

      // Throw away and start over
      resume();
      return ERR;
    }
    void enableIRIn() {
      cli();
      // setup pulse clock timer interrupt
      //Prescale /8 (16M/8 = 0.5 microseconds per tick)
      // Therefore, the timer interval can range from 0.5 to 128 microseconds
      // depending on the reset value (255 to 0)
      TIMER_CONFIG_NORMAL();

      //Timer2 Overflow Interrupt Enable
      TIMER_ENABLE_INTR;

      TIMER_RESET;

      sei();  // enable interrupts
      DDRD |= ~_BV(PORTD1);

      // initialize state machine variables
      irparams.rcvstate = STATE_IDLE;
      irparams.rawlen = 0;

    };
    void resume() {
      irparams.rcvstate = STATE_IDLE;
      irparams.rawlen = 0;
    }
  private:
    // These are called by decode
    int getRClevel(decode_results *results, int *offset, int *used, int t1) {
      if (*offset >= results->rawlen) {
        // After end of recorded buffer, assume SPACE.
        return SPACE;
      }
      int width = results->rawbuf[*offset];
      int val = ((*offset) % 2) ? MARK : SPACE;
      int correction = (val == MARK) ? MARK_EXCESS : - MARK_EXCESS;

      int avail;
      if (MATCH(width, t1 + correction)) {
        avail = 1;
      }
      else if (MATCH(width, 2 * t1 + correction)) {
        avail = 2;
      }
      else if (MATCH(width, 3 * t1 + correction)) {
        avail = 3;
      }
      else {
        return -1;
      }

      (*used)++;
      if (*used >= avail) {
        *used = 0;
        (*offset)++;
      }
      return val;
    };
    long decodeHash(decode_results *results) {
      // Require at least 6 samples to prevent triggering on noise
      if (results->rawlen < 6) {
        return ERR;
      }
      unsigned long hash = FNV_BASIS_32;
      for (int i = 1; i + 2 < results->rawlen; i++) {
        int value =  compare(results->rawbuf[i], results->rawbuf[i + 2]);
        // Add value into the hash
        hash = (hash * FNV_PRIME_32) ^ value;
      }
      results->value = hash;
      return DECODED;
    }
    long decodeRC5(decode_results *results) {
      if (irparams.rawlen < MIN_RC5_SAMPLES + 2) {
        return ERR;
      }
      int offset = 1; // Skip gap space
      long data = 0;
      int used = 0;
      // Get start bits
      if (getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
      if (getRClevel(results, &offset, &used, RC5_T1) != SPACE) return ERR;
      if (getRClevel(results, &offset, &used, RC5_T1) != MARK) return ERR;
      int nbits;
      for (nbits = 0; offset < irparams.rawlen; nbits++) {
        int levelA = getRClevel(results, &offset, &used, RC5_T1);
        int levelB = getRClevel(results, &offset, &used, RC5_T1);
        if (levelA == SPACE && levelB == MARK) {
          // 1 bit
          data = (data << 1) | 1;
        }
        else if (levelA == MARK && levelB == SPACE) {
          // zero bit
          data <<= 1;
        }
        else {
          return ERR;
        }
      }

      // Success
      //results->bits = nbits;
      results->value = data;
      return DECODED;
    }

    int compare(unsigned int oldval, unsigned int newval) {
      if (newval < oldval * .8) {
        return 0;
      }
      else if (oldval < newval * .8) {
        return 2;
      }
      else {
        return 1;
      }
    };

};
ISR(TIMER_INTR_NAME)
{
  TIMER_RESET;
  uint8_t dato = bit_is_set(PIND, PORTD1);
  if (dato > 0) dato = 1;
  uint8_t irdata = dato;

  irparams.timer++; // One more 50us tick
  if (irparams.rawlen >= RAWBUF) {
    // Buffer overflow
    irparams.rcvstate = STATE_STOP;
  }
  switch (irparams.rcvstate) {
    case STATE_IDLE: // In the middle of a gap
      if (irdata == MARK) {
        if (irparams.timer < GAP_TICKS) {
          // Not big enough to be a gap.
          irparams.timer = 0;
        }
        else {
          // gap just ended, record duration and start recording transmission
          irparams.rawlen = 0;
          irparams.rawbuf[irparams.rawlen++] = irparams.timer;
          irparams.timer = 0;
          irparams.rcvstate = STATE_MARK;
        }
      }
      break;
    case STATE_MARK: // timing MARK
      if (irdata == SPACE) {   // MARK ended, record time
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_SPACE;
      }
      break;
    case STATE_SPACE: // timing SPACE
      if (irdata == MARK) { // SPACE just ended, record it
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
      else { // SPACE
        if (irparams.timer > GAP_TICKS) {
          // big SPACE, indicates gap between codes
          // Mark current code as ready for processing
          // Switch to STOP
          // Don't reset timer; keep counting space width
          irparams.rcvstate = STATE_STOP;
        }
      }
      break;
    case STATE_STOP: // waiting, measuring gap
      if (irdata == MARK) { // reset gap timer
        irparams.timer = 0;
      }
      break;
  }
  /*if (irparams.blinkflag) {
  	if (irdata == MARK) {
  		PORTD |= _BV(PORTD3); // turn LED on
  	}
  	else {
  		PORTD &= ~_BV(PORT3); // turn LED off
  	}
    }*/
}




#endif /* IR_RC5_H_ */
