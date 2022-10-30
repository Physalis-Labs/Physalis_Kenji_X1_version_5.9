#ifndef LCD_DISPLAY_H_INCLUDED
#define LCD_DISPLAY_H_INCLUDED

// Functions for communication with display over i2c interface
#include <avr/io.h>
#include <Hardware_Bay.h>
#include <i2c.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <util/delay.h>

//#define LCD_I2C_ADDR           0x4F


//Port Belegung am PCF8574 (1<<PORT)

#define LCD_RS			      (1<<0)
#define LCD_RW			      (1<<1)
#define LCD_E			        (1<<2)
#define LCD_BACKLIGHT	    (1<<3)

#define LCD_D4			      (1<<4)
#define LCD_D5			      (1<<5)
#define LCD_D6			      (1<<6)
#define LCD_D7			      (1<<7)

#define LCD_WIDTH                16
#define LCD_ADDR_LINE1	      (0x80)
#define LCD_ADDR_LINE2	      (0xC0)

void lcd_init(void);

void lcd_clear(void);

void lcd_home(void);

void lcd_nibble_out(unsigned char c, unsigned char cd);

void lcd_print_str (char *str);

void lcd_write_P (const char *Buffer,...);

#define lcd_write(format, args...)   lcd_write_P(PSTR(format) , ## args)

#endif // LCD_DISPLAY_H_INCLUDED
