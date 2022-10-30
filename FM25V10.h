/*------------------------------------------------------------------------------------
- Cypress Semiconductor FRAM Drivers,                                                -
- 1-Mbit (128K × 8) Serial (SPI) F-RAM,                                              -
- SPI BUS read/write speed up to 40 MHz! (^_^)                                       -
- 1-Mbit ferroelectric random access memory (F-RAM) logically organized as 128K × 8  -
- High-endurance 100 trillion (1014) read/writes                                     -
- 151-year data retention                                                            -
- NoDelay™ writes                                                                    -
- Advanced high-reliability ferroelectric process                                    -
- Very fast serial peripheral interface (SPI)                                        -
- Up to 40-MHz frequency                                                             -
- Direct hardware replacement for serial flash and EEPROM                            -
- -----------------------------------------------------------------------            -
- written by Dr.-Ing. David Zohrabyan, Berlin 03.05.2020 at Physalis Labs.           -
------------------------------------------------------------------------------------*/

#include <avr/io.h>
#include <Hardware_Bay.h>

#define FRAM_SELECT                          STEPPER_PORT &= ~(1 << FRAM_SS)
#define FRAM_DESELECT                        STEPPER_PORT |=  (1 << FRAM_SS)

// --------------------- instruction set OPCODES ------------------------- from datasheet -------------------- //

#define FRAM_READ                  0b00000011                                                             // read
#define FRAM_WRITE                 0b00000010                                                            // write
#define FRAM_WRDI                  0b00000100                                                    // write disable
#define FRAM_WREN                  0b00000110                                                     // write enable
#define FRAM_RDSR                  0b00000101                                             // read status register
#define FRAM_WRSR                  0b00000001                                            // write status register

#define FRAM_FSTRD                 0b00001011                                            // fast read memory data
#define FRAM_SLEEP                 0b10111001                                                            // sleep
#define FRAM_READ_ID               0b10011111                                                   // read device ID
#define FRAM_READ_SERIAL_NO        0b11000011                                               // read serial number

// --------------------- FRAM status register bits ---- from datasheet --------------------------------------- //

#define FRAM_WRITE_ENABLE          1                                              // defaults to zero on power-up
                                                                 // WEL = 1 write enabled, WEL = 0 write disabled
#define FRAM_BLOCK_PROTECT_0       2                                                          // block protection
#define FRAM_BLOCK_PROTECT_1       3                                                          // block protection
#define FRAM_WPEN_ENABLE           7                                 // used to enable the (WP) write protect pin

// #define FRAM_BYTES_PER_PAGE     64                                             // 64K locations of 8 data bits
#define FRAM_BYTES_MAX             0xFFFF                           // 1 Mbit FRAM, organized as 131,072 × 8 bits

// ---------------------- FRAM control functions and communication functions  -------------------------------- //

void initSPI(void);                                      // initialize SPI to run FRAM with phase, polarity = 0,0

void SPI_tradeByte(uint8_t byte);              // generic function, just loads up Hardware SPI register and waits

void FRAM_send24BitAddress(uint32_t address);            // splits 24-Bit address into 3 x bytes, sends all three

void FRAM_clearAll(void);                                                    // sets every byte in memory to zero

void FRAM_sleep(void);                                                               // puts FRAM into sleep mode

uint8_t FRAM_readID(void);                                                                     // reads device ID

uint8_t FRAM_SerialNo(void);                                                           // read FRAM serial number

uint8_t FRAM_readStatus(void);                                                  // reads the FRAM status register

void FRAM_writeEnable(void);                                                    // helper: sets FRAM write enable

void FRAM_writeDisable(void);                                                  // helper: sets FRAM write disable

uint8_t FRAM_readByte(uint32_t address);                              // gets a byte from a given memory location

uint8_t FRAM_FastReadByte(uint32_t address);                     // faster byte read from a given memory location

void FRAM_writeByte(uint32_t address, uint8_t byte);                  // writes a byte to a given memory location

uint16_t FRAM_readWord(uint32_t address);                        // gets two bytes from the given memory location

void FRAM_writeWord(uint32_t address, uint16_t word);              // writes two bytes to a given memory location

uint32_t FRAM_readLong(uint32_t address, uint32_t long_word);   // gets more bytes from the given memory location

void FRAM_writeLong(uint32_t address, uint32_t long_word);       // writes 32-Bit long integer to memory location

void FRAM_writeProtectEnable(void);                                           // disables writing to EEPROM array

void FRAM_writeProtectDisable(void);                                           // enables writing to EEPROM array
