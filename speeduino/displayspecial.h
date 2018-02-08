// This is loosly based on the following: [esp8266-Arduino](https://github.com/esp8266/Arduino) library for I2C-OLED displays.

#ifndef DISPLAYSPECIAL_H
#define DISPLAYSPECIAL_H

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

// Font Indices
#define FONT_LENGTH			0
#define FONT_FIXED_WIDTH	2
#define FONT_HEIGHT			3
#define FONT_FIRST_CHAR		4
#define FONT_CHAR_COUNT		5
#define FONT_WIDTH_TABLE	6

class OLED {
	public:
		OLED(uint8_t address=0x3c, uint8_t offset=0);
		void begin(void);
		void on(void);
		void off(void);
		void clear(void);
		void print(char *s, uint8_t r=0, uint8_t c=0);
		void printBigNumber(char *n, uint8_t x=0, uint8_t y=0 );
		void PrintNumberSpecial(int n, uint8_t lineNr);

	private:
		uint8_t _address, _offset;
		void reset_display(void);
		void displayOn(void);
		void displayOff(void);
		void clear_display(void);
		void SendChar(unsigned char data);
		void sendCharXY(unsigned char data, int X, int Y);
		void sendcommand(unsigned char com);
		void setXY(unsigned char col ,unsigned char row);
		void sendStr(unsigned char *string);
		void sendStrXY( const char *string, int X, int Y);
		void init_OLED(void);
};

extern OLED displayspecial;

#endif
