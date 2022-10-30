#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

#include <avr/io.h>
#include <Hardware_Bay.h>

void initI2C(void);             /* Sets pull-ups and initializes bus speed to 100 kHz (at FCPU=8MHz) */

void i2cWaitForComplete(void);  /* Waits until the hardware sets the TWINT flag */

void i2cStart(void);            /* Sends a start condition (sets TWSTA) */

void i2cStop(void);             /* Sends a stop condition (sets TWSTO) */

void i2cSend(uint8_t data);     /* Loads data, sends it out, waiting for completion */

uint8_t i2cReadAck(void);       /* Read in from slave, sending ACK when done (sets TWEA) */

uint8_t i2cReadNoAck(void);     /* Read in from slave, sending NOACK when done (no TWEA) */

#endif // I2C_H_INCLUDED
