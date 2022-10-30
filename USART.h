/***************************************************************************************************************
* ----------------------------------------- =( Physalis Labs. )= --------------------------------------------- *
* ------------------------------------ =( GEO-LAB Robot Firmware )= ------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------ =( Firmware Version 4.3 )= -------------------------------------------- *
* ----------------------------- =( Terrain Exploration Automated System )= ----------------------------------- *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* ---- [ Technical Specifications ] -------------------------------------------------------------------------- *
* ------------------------------------------------------------------------------------------------------------ *
* [MCU/ Controller]: ATmega-1284P-SoC picoPower based on AVR RISC architecture CPU core at 20 MHz              *
* [Propulsion / Power Delivery Unit]: Dual H-Bridge with double 25 x mm geared motors reduction factor 1:150   *
* [Stall current]: 4500 mA (max)                                                                               *
* [Stall Torque]: 9.5 kg NaN                                                                                   *
* [Rated Current]: 1200 mA (Max)                                                                               *
* [Noise]: 56 dB                                                                                               *
* [Working Voltage]: 9 Volt                                                                                    *
* [Step-Motor / Tower Rotation]: 1.8-angle, 2 - PHASE (5 V, 1 A), VEXTA stepping motor                         *
* [Photocells TO-18]: TO-18 series of photo-conductive cells (Cadmium Sulfide)                                 *
* [Vibration Detection Unit]: vibration measurement system based on piezo-electricity                          *
* [Distance Measuring Sensor]: Sharp GP2Y0A02YK0F (20 - 150 cm op.range), (PSD + IRED), analog                 *
* [Speaker]: 5V, DC, Impedance = 8 Ohm, (system Sounds)                                                        *
* [ESP-WROOM-32]: wlan module, Wireless connectivity for remote controlling                                    *
* [Rechargeable Battery 14 Volt]: Power Supply for Robot platform, monitored by AVR                            *
* [EEPROM]: Microchip 256 kByte EEPROM for non-volatile memory, and data storage                               *
* ------------------------------------------------------------------------------------------------------------ *
* [Header file]: "SHARP Infrared distance sensor" defining all functionality                                   *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* [written by]: Zohrabyan David at Physalis Labs. [Berlin 08.07.2019], Richardstrasse 110, 12043 ------------- *
***************************************************************************************************************/
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//----------------------------------- [ Header File IRED-PSD Distance Sensor] --------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
/*

 Functions to initialize, send, receive over USART
 initUSART requires BAUD to be defined in order to calculate the bit-rate multiplier.

*/

#ifndef BAUD                                               /* if not defined in Makefile... */
#define BAUD  9600                                          /* set a safe default baud rate */
#endif

#define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)        /* These are defined for convenience */
#define   USART_READY      bit_is_set(UCSR0A, UDRE0)

void initUSART(void);                               /*   Takes the defined BAUD and F_CPU, calculates the bit-clock multiplier, and configures the hardware USART */


void transmitByte(uint8_t data);                    /* Blocking transmit and receive functions. When you call receiveByte() your program will hang until data comes through.  We'll improve on this later. */

uint8_t receiveByte(void);

void printString(const char myString[]);                    /* Utility function to transmit an entire string from RAM */

void readString(char myString[], uint8_t maxLength);        /* Define a string variable, pass it to this function The string will contain whatever you typed over serial */

void printByte(uint8_t byte);                              /* Prints a byte out as its 3-digit ascii equivalent */

void printWord(uint16_t word);                             /* Prints a word (16-bits) out as its 5-digit ascii equivalent */

void printWord_32(uint32_t word);                          /* Prints a word (32-bits) out as its 5-digit ascii equivalent */

void printBinaryByte(uint8_t byte);                        /* Prints a byte out in 1s and 0s */

char nibbleToHex(uint8_t nibble);

char nibbleToHexCharacter(uint8_t nibble);

void printHexByte(uint8_t byte);                          /* Prints a byte out in hexadecimal */

uint8_t  getNumber(void);                                 /* takes in up to three ascii digits, converts them to a byte (8-bit) when press enter */

uint16_t get_16BitNumber(void);                           /* takes in up to five ascii digits, converts them to a word (16-bit) when press enter */

uint32_t get_32BitNumber(void);                           /* takes in up to ten ascii digits, converts them to a long (32-bit) when press enter */
