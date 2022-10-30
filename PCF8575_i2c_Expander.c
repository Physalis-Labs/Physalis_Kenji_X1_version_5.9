/*------------------------------------------------------------------------------------
- PCF8575 Remote 16-Bit(I2C) GPIO expander with interrupt, Texas Instruments.        -
- Datasheet info: SCPS121H–JANUARY 2005–REVISED FEBRUARY 2020.                       -
- 16 GPIO are available for general purpose tasks.                                   -
- 400-kHz Fast I2C Bus! (^_^).                                                       -
- 16-bit I/O expander for the two-line bidirectional bus (I2C).                      -
- Open-Drain Interrupt Output for the (MCU).                                         -
- 16-bit quasi-bidirectional input/output (I/O) ports (P07–P00, P17–P10).            -
- Latched outputs with high-current drive capability for directly driving LEDs.      -
- Latch-Up Performance Exceeds 100 mA per JESD 78, Class II                          -
- Designed for 2.5-V to 5.5-V VCC operation                                          -
- -------------------------------------------------------------------------          -
- written by Dr.-Ing. David Zohrabyan, Potsdam 11.02.2021 at Physalis Labs.          -
------------------------------------------------------------------------------------*/

#include <PCF_8575.h>
#include <USART.h>

void set_pcf8575_GPIO_HIGH(void){   // we make sure to setup PCF8575 all GPIO to (1) before starting, input mode, and before reading the inputs

    i2cStart();
    i2cSend(PCF8575_ADDRESS_W);
    i2cSend(0b11111111);
    i2cSend(0b11111111);
    i2cStop();
}

void pcf8575_Output(uint16_t data){ // function to communicate with PCF8575 output on suitable GPIOs.
                                    // consult device datasheet if input and output combined with loads are used.
    uint8_t data_highbyte, data_lowbyte;
    data_lowbyte = (data >> 8);
    data_highbyte = (uint8_t)data;

    i2cStart();
    i2cSend(PCF8575_ADDRESS_W);
    i2cSend(data_highbyte);
    i2cSend(data_lowbyte);
    i2cStop();
}

                                    // function to connect to PCF8575 GPIO expander and readout data available on suitable GPIOs
uint16_t pcf8575_Input(void){

    uint8_t readin_highbyte;
    uint8_t readin_lowbyte;
    uint16_t incoming_data;
    uint16_t reg_bitmask = 0b1111111111111111;

    i2cStart();                     // seconds start and now we are ready to read the data from GPIOs
    i2cSend(PCF8575_ADDRESS_R);

    readin_lowbyte =  i2cReadAck();
    readin_highbyte = i2cReadAck();
    i2cStop();
    incoming_data = ((reg_bitmask & readin_highbyte) << 8);
    printWord(incoming_data);
    printString("\r\n");
    incoming_data = incoming_data | (uint16_t)readin_lowbyte;
    printWord(incoming_data);
    printString("\r\n");
    return(incoming_data);
}
