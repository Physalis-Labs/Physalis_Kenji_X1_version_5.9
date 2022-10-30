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
* [Propulsion / Power Delivery Unit]: Dual H-Bridge with double 25 x mm geared motors reduction factor 1:150    *
* [Stall current]: 4500 mA (max)                                                                               *
* [Stall Torque]: 9.5 kg NaN                                                                                   *
* [Rated Current]: 1200 mA (Max)                                                                               *
* [Noise]: 56 dB                                                                                               *
* [Working Voltage]: 9 Volt                                                                                    *
* [Step-Motor / Tower Rotation]: 1.8-angle, 2 - PHASE (5 V, 1 A), VEXTA stepping motor                         *
* [Photocells TO-18]: TO-18 series of photo-conductive cells (Cadmium Sulfide)                                 *
* [Vibration Detection Unit]: vibration measurement system based on piezo-electricity                           *
* [Distance Measuring Sensor]: Sharp GP2Y0A02YK0F (20 - 150 cm op.range), (PSD + IRED), analog                 *
* [Speaker]: 5V, DC, Impedance = 8 Ohm, (system Sounds)                                                        *
* [ESP-WROOM-32]: wlan module, Wireless connectivity for remote controlling                                    *
* [Rechargeable Battery 14 Volt]: Power Supply for Robot platform, monitored by AVR                             *
* [EEPROM]: Microchip 256 kByte EEPROM for non-volatile memory, and data storage                               *
* ------------------------------------------------------------------------------------------------------------ *
* [Source file]: "battery_pack.c" running functions of [ATmega-1284P] and power supply control sub-system      *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* [written by]: Zohrabyan David at Physalis Labs. [Berlin 08.07.2019], Richardstrasse 110, 12043 ------------- *
***************************************************************************************************************/
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//----------------------------------- [ Source File battery-pack ] -------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//

#include <avr/io.h>
#include <Hardware_Bay.h>
#include <battery_pack.h>
#include <avr/sleep.h>

float power_pack_voltage;

void setup_ADC_sleepmode(void){                                   // halts CPU until ADC measurement in progress

    set_sleep_mode(SLEEP_MODE_ADC);                                               // defined in avr/sleep.h file
    ADCSRA |= (1 << ADIE);                                                           // enable ADC interrupts...
}

uint16_t fetchDataADC(uint8_t channel){

    ADMUX = ADMUX & 0b11100000;                                    // Reset the Multiplexer and disconnect GPIOs
    ADMUX = ADMUX | channel;                                      // Connect ADC hardware to multiplexer channel
    ADCSRA |= (1 << ADSC);                                                               // ADC start conversion
    do{} while ((ADCSRA & (1 << ADSC)) == 0);                                   // wait until conversion is done
        return(ADC);                                                     // read conversion result into variable
}

uint16_t oversample16x(void){

    uint16_t oversampled_value = 0;
    uint8_t i;

    for(i = 0; i < 16; i ++){
        sleep_mode();                                                         // sleep mode for CPU... ADC runs
        oversampled_value += fetchDataADC(CHARGE);                            // add the 16 ADC values together
    }
    return(oversampled_value >> 2);                                                    // divide back down by 4
}
