/*********************************************************************************************************
* ------------------------------- =( Physalis Labs. )= ------------------------------------------------- *
* ------------------------------- =( Banshee Robot Firmware 4.3 )= ------------------------------------- *
* ------------------------------------------------------------------------------------------------------ *
* -------------------------- =( Terrain Exploration Automated Systems )= ------------------------------- *
* ------------------------------------------------------------------------------------------------------ *
* [ATmega-1284P]-SoC picoPower 8-bit microcontroller based on AVR RISC architecture CPU runs at 26 MHz   *
* [Header file]: populating the hardware physical connections to and from AVR SoC                        *
* [Data structure]: which lays down the Program Firmware skeleton                                        *
* -------------------------------------------------------------------------------------------------------*
* [written by]: Zohrabyan David at Physalis Labs. [Berlin 08.07.2019], Richardstrasse 110, 12043 ------- *
*********************************************************************************************************/
//------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------//
//-------------------------- [ ATmega-1284P PORT INPUT-OUTPUT INFRASTRUCTURE ] -------------------------//
//------------------------------------------------------------------------------------------------------//

//------------------------------- PORT A Data Register -------------------------------------------------//
#include <avr/io.h>

#define CTRL_DDR                DDRA   // left/right BRIDGE  low-side control GPIO
#define CTRL_PORT               PORTA
#define CTRL_PIN                PINA

#define RBN3_FET                PA0    // right bridge left bottom MOSFET/ N-3
#define RBN4_FET                PA1    // right bridge right bottom MOSFET/ N-4
#define LBN7_FET                PA2    // left bridge right bottom MOSFET/ N-7
#define LBN8_FET                PA3    // left bridge left bottom MOSFET/ N-8

#define AMBIENT_LIGHT           PA4    // Cadmium-Sulfide light sensor / Tower-TOP (ambient light sensing)
#define VIBRATION_SENSOR        PA5    // vibration sensor, made from piezoelectric crystal
#define IRED_SENSOR             PA6    // Sharp short-range (20 - 150 cm), infrared distance sensor (tower mount)
#define CHARGE                  PA7    // Power Supply pack voltage monitoring ADC channel

//------------------------------- Port B Data Register -------------------------------------------------//

#define STEPPER_DDR             DDRB
#define STEPPER_PORT            PORTB
#define STEPPER_PIN             PINB

#define COIL_A1                 PB0    // step motor COIL-1 control one-half
#define COIL_A2                 PB1    // step motor COIL-1 control second-half
//#define SWITCH_TTL              PB2    // free at the moment, can be used for pretty much anything ..... David
#define SPEAKER                 PB3    // mini-speaker (+) terminal hardware connection

//#define EEPROM_SS               PB4    // off-board SPI EEPROM SLAVE SELECT (Microchip "25LC256 256 kByte Serial EEPROM")
//#define EEPROM_MOSI             PB5    // off-board SPI EEPROM MOSI LINE
//#define EEPROM_MISO             PB6    // off-board SPI EEPROM MISO LINE
//#define EEPROM_SCK              PB7    // off-board SPI EEPROM CLOCK LINE

#define FRAM_SS               PB4    // off-board SPI FRAM SLAVE SELECT (Cypress Semiconductor 2MBit FRAM FM25V10/FM25V20)
#define FRAM_MOSI             PB5    // off-board SPI FRAM MOSI LINE
#define FRAM_MISO             PB6    // off-board SPI FRAM MISO LINE
#define FRAM_SCK              PB7    // off-board SPI FRAM CLOCK LINE
//------------------------------- Port C Data Register -------------------------------------------------//

#define HALL_DDR                DDRC
#define HALL_PORT               PORTC
#define HALL_PIN                PINC

#define I2C_SDA_LINE            PC0    // AVR I2C-line, SCL (serial clock)
#define I2C_SCL_LINE            PC1    // AVR I2C-line, SDA (serial data)
#define HALL_S1R                PC2    // right motor S1 Hall effect sensor / move_knob
#define HALL_S2R                PC3    // right motor S2 Hall effect sensor / move_dir
#define HALL_S1L                PC4    // left motor S1 Hall effect sensor  / move_knob
#define HALL_S2L                PC5    // left motor S2 Hall effect sensor  / move_dir
#define COIL_B1                 PC6    // step motor COIL-2 control one-half
#define COIL_B2                 PC7    // step motor COIL-2 control second-half

//------------------------------- Port D Data Register -------------------------------------------------//

#define CTRLPWM_DDR             DDRD   // left/right BRIDGE up-side PWM control GPIO
#define CTRLPWM_PORT            PORTD
#define CTRLPWM_PIN             PIND

#define RX_COMM                 PD0   // reserved for RX data COMM / HOST PC link
#define TX_COMM                 PD1   // reserved for TX data COMM / HOST PC link
#define GREED_LED               PD2   // system status led, shows program execution rollover(green) inside tower window
#define POWER_ENABLE_LIDAR      PD3   // power enable GPIO to control the Garmin LIDAR Lite_V3.(On/Off)
#define RBN1_FET                PD4   // right bridge left top MOSFET/ N-1 /PWM-line
#define RBN2_FET                PD5   // right bridge right top MOSFET/ N-2 /PWM-line
#define LBN5_FET                PD6   // left bridge left top MOSFET/ N-5 /PWM-line
#define LBN6_FET                PD7   // left bridge right top MOSFET/ N-6 /PWM-line
