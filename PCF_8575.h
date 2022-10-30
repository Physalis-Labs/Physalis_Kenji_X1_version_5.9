/*------------------------------------------------------------------------------------
- PCF8575 Remote 16-Bit(I2C) GPIO expander with interrupt output drivers,            -
- 16 GPIO available for general purpose tasks,                                       -
- 400-kHz Fast I2C Bus! (^_^),                                                       -
- 16-bit I/O expander for the two-line bidirectional bus (I2C)                       -
- Open-Drain Interrupt Output                                                        -
- 16-bit quasi-bidirectional input/output (I/O) port (P07–P00, P17–P10)              -
- latched outputs with high-current drive capability for directly driving LEDs.      -
- Latch-Up Performance Exceeds 100 mA per JESD 78, Class II                          -
- Designed for 2.5-V to 5.5-V VCC operation                                          -
- -----------------------------------------------------------------------            -
- written by Dr.-Ing. David Zohrabyan, Potsdam 11.02.2021 at Physalis Labs.          -
------------------------------------------------------------------------------------*/
#ifndef PCF_8575_H_INCLUDED
#define PCF_8575_H_INCLUDED

#include <avr/io.h>
#include <Hardware_Bay.h>
#include <i2c.h>

#define PCF8575_ADDRESS_W     0b01000000             // to write to device with (0) at the end
#define PCF8575_ADDRESS_R     0b01000001  // to read from the device ports with (1) at the end

//----------------- I2C Remote GPIO Expander input/output infrastructure -------------------//

//----------------- free GPIO can be used for anything -------------------------------------//

      // ------------ low byte ------------//
#define PCF_GPIO_P00                   0 //P00
#define PCF_GPIO_P01                   1 //P01
#define PCF_GPIO_P02                   2 //P02
#define PCF_GPIO_P03                   3 //P03
#define PCF_GPIO_P04                   4 //P04
#define PCF_GPIO_P05                   5 //P05
#define PCF_GPIO_P06                   6 //P06
#define PCF_GPIO_P07                   7 //P07

//----------------- output GPIO setup to drive BAR-Graph LEDs ------------------------------//

      // ------------ high byte ------------//
#define LED_0                         8  //P10
#define LED_1                         9  //P11
#define LED_2                         10 //P12
#define LED_3                         11 //P13
#define LED_4                         12 //P14
#define LED_5                         13 //P15
#define LED_6                         14 //P16
#define LED_7                         15 //P17

#define ALL_LED_OFF                   0b0000000000000000
#define ALL_LED_ON                    0b1111111111111111

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */

void pcf8575_Output(uint16_t data);      // function to connect to PCF8575 GPIO expander and output data on suitable GPIOs


/** \brief
 *
 * \param
 * \param
 * \return
 *
 */

uint16_t pcf8575_Input(void); // function to connect to PCF8575 GPIO expander and readout data available on suitable GPIOs

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */

void set_pcf8575_GPIO_HIGH(void); // we make sure to setup PCF8575 all GPIO to (1) before starting, input mode, and before reading the inputs

#endif // PCF_8575_H_INCLUDED
