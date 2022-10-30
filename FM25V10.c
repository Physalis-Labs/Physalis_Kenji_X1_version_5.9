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

#include <FM25V10.h>
#include <Hardware_Bay.h>

void initSPI(void){

  STEPPER_DDR |= (1 << FRAM_SS);                      // set SPI_FRAM as output
  STEPPER_PORT |= (1 << FRAM_SS);         // start off not selected, logic high

  STEPPER_DDR |= (1 << FRAM_MOSI);                       // setup MOSI as output
  STEPPER_PORT |= (1 << FRAM_MISO);                  // activate pull-up on MISO
  STEPPER_DDR |= (1 << FRAM_SCK);                          // setup SCK as output

// ------ no need to set phase, polarity bc, default works with FM25V10 ----- //

  SPCR |= (1 << SPR0);               // prescaler fosc/div-16; 500 kHz BUS SPEED
  SPCR |= (1 << MSTR);                                            // clockmaster
  SPCR |= (1 << SPE);                                     // enable SPI hardware
}

void SPI_tradeByte(uint8_t byte){

  SPDR = byte;                                 // SPI starts sending immediately
  while((SPSR & (1 << SPIF)) == 0)                      // wait until bit is set
  {;}                                        // SPDR0 now contains received byte
}

void FRAM_send24BitAddress(uint32_t address){

  SPI_tradeByte( (uint8_t) (address >> 16) );       // MSB most significant byte
  SPI_tradeByte( (uint8_t) (address >> 8) );                      // Middle Byte
  SPI_tradeByte( (uint8_t) (address));             // LSB least significant byte
}

uint8_t FRAM_SerialNo(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRSR);
  SPI_tradeByte(FRAM_READ_SERIAL_NO);
  SPI_tradeByte(0);
  FRAM_DESELECT;
  return(SPDR);
}

uint8_t FRAM_readStatus(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_RDSR);
  SPI_tradeByte(0);                                          // clock out 8 bits
  FRAM_DESELECT;
  return(SPDR);                                                // return results
}

void FRAM_writeEnable(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_WREN);
  FRAM_DESELECT;
}

void FRAM_writeDisable(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRDI);
  FRAM_DESELECT;
}

uint8_t FRAM_readByte(uint32_t address){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_READ);
  FRAM_send24BitAddress(address);
  SPI_tradeByte(0);
  FRAM_DESELECT;
  return(SPDR);
}

uint8_t FRAM_FastReadByte(uint32_t address){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_FSTRD);
  FRAM_send24BitAddress(address);
  SPI_tradeByte(0);
  SPI_tradeByte(0);
  FRAM_DESELECT;
  return(SPDR);
}


void FRAM_writeByte(uint32_t address, uint8_t byte ){

  FRAM_writeEnable();
  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRITE);
  FRAM_send24BitAddress(address);
  SPI_tradeByte(byte);
  loop_until_bit_is_set(SPSR, SPIF);
  FRAM_DESELECT;
}

uint16_t FRAM_readWord(uint32_t address){

  uint16_t fram_Word;
  FRAM_SELECT;
  SPI_tradeByte(FRAM_READ);
  FRAM_send24BitAddress(address);
  SPI_tradeByte(0);
  fram_Word = SPDR;                                             // most sig. bit
  fram_Word = (fram_Word << 8);                                // least sig. bit
  SPI_tradeByte(0);
  fram_Word += SPDR;
  FRAM_DESELECT;
  return(fram_Word);
}

void FRAM_writeWord(uint32_t address, uint16_t word){

  FRAM_writeEnable();
  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRITE);
  FRAM_send24BitAddress(address);
  SPI_tradeByte((uint8_t) (word >> 8));                         // most sig. bit
  SPI_tradeByte((uint8_t) word);                               // least sig. bit
  FRAM_DESELECT;
}

// uint32_t FRAM_readLong(uint32_t address){
//
//   uint32_t fram_long_word:
//   FRAM_writeEnable();
//   FRAM_SELECT;
//   SPI_tradeByte(FRAM_READ);
//   FRAM_send24BitAddress(address);
//   SPI_tradeByte(0);
//   fram_long_word = SPDR;
//   fram_long_word = (fram_long_word << 16);
//   SPI_tradeByte(0);
//   fram_long_word = (fram_long_word << 8);
//
//   loop_until_bit_is_set(SPSR, SPIF);
//   FRAM_DESELECT;
// }
//
// void FRAM_writeLong(uint32_t address, uint32_t long_word ){
//
//   FRAM_writeEnable();
//   FRAM_SELECT;
//   SPI_tradeByte(FRAM_WRITE);
//   FRAM_send24BitAddress(address);
//   SPI_tradeByte( (uint8_t) (long_word >> 16) );
//   SPI_tradeByte( (uint8_t) (long_word >> 8) );
//   SPI_tradeByte( (uint8_t) (long_word) );
//   loop_until_bit_is_set(SPSR, SPIF);
//   FRAM_DESELECT;
// }

void FRAM_clearAll(void) {

  uint32_t Address = 0;
  while(Address < FRAM_BYTES_MAX) {
    FRAM_writeEnable();
    FRAM_SELECT;
    SPI_tradeByte(FRAM_WRITE);
    FRAM_send24BitAddress(Address);
    SPI_tradeByte(0);
    Address ++ ;
    FRAM_DESELECT;
  }
}

void FRAM_writeProtectEnable(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRSR);
  SPI_tradeByte(FRAM_BLOCK_PROTECT_0);
  SPI_tradeByte(FRAM_BLOCK_PROTECT_1);
  FRAM_DESELECT;
}

void FRAM_writeProtectDisable(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRSR);
  SPI_tradeByte(0);
  SPI_tradeByte(0);
  FRAM_DESELECT;
}

uint8_t FRAM_readID(void){

  FRAM_SELECT;
  SPI_tradeByte(FRAM_WRSR);
  SPI_tradeByte(FRAM_READ_ID);
  SPI_tradeByte(0);
  FRAM_DESELECT;
  return(SPDR);
}
