/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "deftype.h"

#define PROGMEM

#define __asm__ 
#define __volatile__(B)

#ifdef __cplusplus
extern "C"{
#endif

extern unsigned int millis            (void) ;
extern unsigned int micros            (void) ;

void yield(void);


#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define INTERNAL1V1 2
#define INTERNAL2V56 3

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))


#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


//typedef unsigned int word;

#define bit(b) (1UL << (b))


/*
void init(void);
void initVariant(void);

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

void setup(void);
void loop(void);

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)

// On the ATmega1280, the addresses of some of the port registers are
// greater than 255, so we can't store them in uint8_t's.
extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];

extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_bit_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

#define NOT_A_PIN 0
#define NOT_A_PORT 0

#define NOT_AN_INTERRUPT -1

#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#endif

#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1C 5
#define TIMER2  6
#define TIMER2A 7
#define TIMER2B 8

#define TIMER3A 9
#define TIMER3B 10
#define TIMER3C 11
#define TIMER4A 12
#define TIMER4B 13
#define TIMER4C 14
#define TIMER4D 15
#define TIMER5A 16
#define TIMER5B 17
#define TIMER5C 18

*/
#ifdef __cplusplus
} // extern "C"
#endif

/*
uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned long);
long map(long, long, long, long, long);

#include "pins_arduino.h"

*/

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/*
class Print
{
public:
	char buffer[1024];
	int buflen;

	int PRINT(long mes, int base, bool lf)
	{
        int nb=0;
		switch (base) {
		case BIN:  nb=printf("%d", mes);
			break;
		case OCT:  nb=printf("%o", mes);
			break;
		case DEC:  nb=printf("%d", mes);
			break;
		case HEX:  nb=printf("%X", mes);
			break;
		default:   nb=printf("%d", mes);
			break;
		}
		if (lf)printf("\n");
		return nb;
	}


	int write(byte mes) { printf("%02X", mes); return 1; };
	int write(char mes) { printf("%C", mes); return 1; };

	int write(const byte *buffer, int size) {
		for (int i = 0; i<size; i++) write(buffer[i]);
        return size;
	}

	int write(const byte *str) {
		if (str == NULL) return 0;
		return write(str, strlen((char*)str));
	}

	 int print(const char mes[]) { return printf("%s", mes); };
	 int print(char mes) { return printf("%c", mes); };
	 int print(unsigned char mes, int base = DEC) { return PRINT(mes, base, false); };
	 int print(int mes, int base = DEC) { return PRINT(mes, base, false); };
	 int print(unsigned int mes, int base = DEC) { return PRINT(mes, base, false); };
	 int print(long mes, int base = DEC) { return PRINT(mes, base, false); };
	 int print(unsigned long mes, int base = DEC) { return PRINT(mes, base, false); };
	 int print(double mes, int base = 2) { return printf("%f", mes); };
	 int println(const char mes[]) { return printf("%s\n", mes); };
	 int println(char mes) { return printf("%c\n", mes); };
	 int println(unsigned char mes, int base = DEC) { return PRINT(mes, base, true); };
	 int println(int mes, int base = DEC) { return PRINT(mes, base, true); };
	 int println(unsigned int mes, int base = DEC) { return PRINT(mes, base, true); };
	 int println(long mes, int base = DEC) { return PRINT(mes, base, true); };
	 int println(unsigned long mes, int base = DEC) { return PRINT(mes, base, true); };
	 int println(double mes, int base = 2) { return printf("%f\n", mes); };
	 int println(void) { return printf("\n"); };


    int available() 
    {
	    return buflen;
    };

     char read() 
    { 
	    char c = 0;
	    if (buflen > 0)
	    {
		    c = buffer[0];
		    memcpy(&buffer[0], &buffer[1], --buflen);
	    }
	    return c;
    };
    void _append(char c)
    {
	    buffer[buflen] = c;
	    if (++buflen >= 1024)
	    {
		    buflen--;
	    }
    };





    void begin(long){ };

};

extern void cli();
extern void sei();

extern Print Serial;
*/

static byte digitalPinToInterrupt(byte pint)
{
	return pint;
}

#define LED_BUILTIN 0

#include "print.h"


#endif
