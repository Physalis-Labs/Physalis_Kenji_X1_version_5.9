#include "i2c.h"

void initI2C(void) {

  HALL_PORT |= ((1 << I2C_SDA_LINE) | (1 << I2C_SCL_LINE));    /* set pull-ups for SDA, SCL lines */
  TWBR = 70;                                                    /* set bit rate (p.242): 20 MHz / (16+2*TWBR*1)= ~ 715 kHz */
  //TWSR |= (1 << TWPS0);                                      // I-2C clock prescaler div / 4
  TWCR |= (1 << TWEN);                                         /* enable */
}

void i2cWaitForComplete(void) {

  loop_until_bit_is_set(TWCR, TWINT);
}

void i2cStart(void) {

  TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWSTA));
  i2cWaitForComplete();
}

void i2cStop(void) {

  TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWSTO));
}

uint8_t i2cReadAck(void) {

  TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWEA));
  i2cWaitForComplete();
  return (TWDR);
}

uint8_t i2cReadNoAck(void) {

  TWCR = (_BV(TWINT) | _BV(TWEN));
  i2cWaitForComplete();
  return (TWDR);
}

void i2cSend(uint8_t data) {

  TWDR = data;
  TWCR = (_BV(TWINT) | _BV(TWEN));                  /* init and enable */
  i2cWaitForComplete();
}
