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
* [Source file]: "vibration_sensor.c" running functions of [ATmega-1284P] and vibration sensor sub-system      *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* [written by]: Zohrabyan David at Physalis Labs. [Berlin 08.07.2019], Richardstrasse 110, 12043 ------------- *
***************************************************************************************************************/
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//----------------------------------- [ Source File vibration sensor ] ---------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------ working variables -----------------------------------------------------//
#include <avr/io.h>
#include <Hardware_Bay.h>
#include <vibration_sensor.h>
#include <battery_pack.h>
#include <USART.h>

uint16_t lightsOutTimer = 0;                                                    // timer for the switch, on PB2
uint16_t adcValue;

uint16_t middleValue = 511;
uint16_t highValue = 520;
uint16_t lowValue = 500;

uint16_t noiseVolume = 0;
uint8_t padding = INITIAL_PADDING;
uint32_t theta_0, theta_1;                                               // time difference calculation variables
//---------------------------------------- functions ----------------------------------------------------------//
void measure_Vibrations(void){

  TCNT3 = 0;
  theta_0 = 0;

  adcValue = fetchDataADC(VIBRATION_SENSOR);

  middleValue = adcValue + middleValue - ((middleValue - 8) >> 4);// moving average: tracks sensor'r bias voltage

  if (adcValue > (middleValue >> 4)){            // moving averages for positive and negative parts of the signal
    highValue = adcValue + highValue - ((highValue - 8) >> 4);
  }

  if (adcValue < (middleValue >> 4)){
    lowValue = adcValue + lowValue - ((lowValue - 8) >> 4);
  }

  noiseVolume = highValue + lowValue + padding;        // "padding" provides a minimum value for the noise volume

// ----------------- checking to see if ADC value, above or below the thresholds ----------------------------- //
// ----------------------------------------------------------------------------------------------------------- //
// ------------------------- comparison with >> 4 b/c EMWA is on different scale ----------------------------- //

  if (adcValue < ((middleValue - noiseVolume) >> 4)){
    //STEPPER_PORT  = (1 << RED_LED_1) | (1 << SWITCH);                    // turn on RED_LED_1, and the "switch"
    lightsOutTimer = ON_TIME / CYCLE_DELAY;                                                        // reset timer
  }

  else if (adcValue > ((middleValue + noiseVolume) >> 4)){
//HALL_PORT = (1 << RED_LED_2) | (1 << RED_LED_3) | (1 << SWITCH);    // turn-on RED_LED_2,RED_LED_3 and "switch"
    lightsOutTimer = ON_TIME / CYCLE_DELAY;                                                        // reset timer
  }

  else {
    //STEPPER_PORT &= ~(1 << RED_LED_1);                                                             // both OFF!
    //HALL_PORT &= ~((1 << RED_LED_2) | (1 << RED_LED_3));                                            // all OFF!

    if(lightsOutTimer > 0){
      lightsOutTimer -- ;
    }
    else{                                                                                            // time's up
      //STEPPER_PORT |= (1 << SWITCH);                                                   // turn off the "SWITCH"
    }
  }

  theta_0 = (TCNT3 * 51) / 1000;                                                          // time in milliseconds
  printString("ADC-(adcValue) = ");
  printByte(adcValue - 512 + 127);

  printString("\t");
  printString("ADC-(lowValue) = ");                              // ADC is 10-bits, recenter for display purposes
  printByte((lowValue >> 4) - 512 + 127);
  printString("\t");
  printString("ADC-(highValue) = ");
  printByte((highValue >> 4) - 512 + 127);
  printString("\t");
//  printString("time = ");
//  printWord(TCNT3);
//  printString("\r\n");
}
