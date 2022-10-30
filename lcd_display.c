#include <lcd_display.h>
#include <i2c.h>

#define LCD_I2C_ADDR                0b01001110

volatile unsigned char char_counter = 0;

// output data to lcd
void lcd_out (unsigned char c) {
	i2cStart();
    i2cSend(LCD_I2C_ADDR);
	i2cSend(c | LCD_BACKLIGHT);
	i2cStop();
}

//***************************************************************************************
// CD = Command or Data
void lcd_nibble_out(unsigned char c, unsigned char cd) {
	unsigned char out = 0;
	unsigned char rs = 0;

	if(cd) rs=LCD_RS;

	//4 upper bits
	if(c & (1<<4)) out |= LCD_D4;
	if(c & (1<<5)) out |= LCD_D5;
	if(c & (1<<6)) out |= LCD_D6;
	if(c & (1<<7)) out |= LCD_D7;
	lcd_out(out | rs | LCD_E);
	_delay_ms(5);
	lcd_out(out | rs);

	//4 lower bits
	out=0;
	if(c & (1<<0)) out |= LCD_D4;
	if(c & (1<<1)) out |= LCD_D5;
	if(c & (1<<2)) out |= LCD_D6;
	if(c & (1<<3)) out |= LCD_D7;
	lcd_out(out | rs | LCD_E);
	_delay_ms(5);
	lcd_out(out | rs );
}

//***************************************************************************************
// clear LCD
void lcd_clear() {
	lcd_nibble_out(0x01, 0); // clear display
	lcd_nibble_out(0x80, 0);
	char_counter = 0;
}


//***************************************************************************************
// LCD home
void lcd_home() {
	lcd_nibble_out(0x80, 0);
	char_counter = 0;
}

//***************************************************************************************
// Init LCD
void lcd_init() {
	unsigned char loop=3;

	while(loop--){
		lcd_out(LCD_D4 | LCD_D5 | LCD_E);
		_delay_ms(10);
		lcd_out(LCD_D4 | LCD_D5);
		_delay_ms(100);
	}

	// 4 bit mode
	lcd_out(LCD_D5 | LCD_E);
	_delay_ms(10);
	lcd_out(LCD_D5);
	_delay_ms(10);

	lcd_nibble_out(0x28, 0);
	lcd_nibble_out(0x0C, 0);

	lcd_clear();
}

//***************************************************************************************
void lcd_write_char (char c) {
	if(char_counter == LCD_WIDTH) lcd_nibble_out(LCD_ADDR_LINE2,0);
	if(char_counter == (LCD_WIDTH*2)){
		lcd_nibble_out(LCD_ADDR_LINE1,0);
		char_counter = 0;
	}
	char_counter++;
	lcd_nibble_out(c, 1);
}

//***************************************************************************************
void lcd_print_str(char *str) {
	while (*str != 0){
		if(char_counter == LCD_WIDTH) lcd_nibble_out(LCD_ADDR_LINE2,0);
		if(char_counter == (LCD_WIDTH*2)){
			lcd_nibble_out(LCD_ADDR_LINE1,0);
			char_counter = 0;
		}
		char_counter++;
		lcd_nibble_out(*str++, 1);
	}
}

//***************************************************************************************
void lcd_write_P (const char *Buffer,...)
{
	va_list ap;
	va_start (ap, Buffer);

	int format_flag;
	char str_buffer[10];
	char str_null_buffer[10];
	char move = 0;
	char Base = 0;
	int tmp = 0;
	char by;
	char *ptr;

	//Ausgabe der Zeichen
    for(;;){
		by = pgm_read_byte(Buffer++);
		if(by==0) break; // end of format string

		if (by == '%'){
            by = pgm_read_byte(Buffer++);
			if (isdigit(by)>0){
 				str_null_buffer[0] = by;
				str_null_buffer[1] = '\0';
				move = atoi(str_null_buffer);
                by = pgm_read_byte(Buffer++);
				}

			switch (by){
                case 's':
                    ptr = va_arg(ap,char *);
                    while(*ptr) { lcd_write_char(*ptr++); }
                    break;
				case 'b':
					Base = 2;
					goto ConversionLoop;
				case 'c':
					//Int to char
					format_flag = va_arg(ap,int);
					lcd_write_char (format_flag++);
					break;
				case 'i':
					Base = 10;
					goto ConversionLoop;
				case 'o':
					Base = 8;
					goto ConversionLoop;
				case 'x':
					Base = 16;
					//****************************
					ConversionLoop:
					//****************************
					itoa(va_arg(ap,int),str_buffer,Base);
					int b=0;
					while (str_buffer[b++] != 0){};
					b--;
					if (b<move){
						move -=b;
						for (tmp = 0;tmp<move;tmp++){
							str_null_buffer[tmp] = '0';
							}
						//tmp ++;
						str_null_buffer[tmp] = '\0';
						strcat(str_null_buffer,str_buffer);
						strcpy(str_buffer,str_null_buffer);
						}
					lcd_print_str (str_buffer);
					move =0;
					break;
				}
		}else{
			lcd_write_char (by);
		}
	}
	va_end(ap);
}
