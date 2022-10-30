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
#ifndef SHARP_IRED_PSD_SENSOR_H_INCLUDED
#define SHARP_IRED_PSD_SENSOR_H_INCLUDED

#define REF_VCC                    5.061       // ADC voltage measured on AVCC pin

// ---------------------- Sharp GP2Y0A02YK0F (20 - 150 cm) model ---------------------------------------------//

#define COEFF_A                 10650.08       // taken from fitting of the function curve provided in datasheet
#define COEFF_B                   -0.935       // taken from fitting of the function curve provided in datasheet
#define COEFF_C                     10.0       // taken from fitting of the function curve provided in datasheet

// ---------------------- Sharp GP2Y0A710K0F (100 - 550 cm) model ---------------------------------------------//

#define COEFF_D                   1125         // taken from fitting of the function curve provided in datasheet
#define COEFF_E                   137500       // taken from fitting of the function curve provided in datasheet
#define COEFF_F                   1000         // divide by 1000 to rescale from Volts to miliVolts

float measureDistance(uint16_t analog_value);  // measures distance from object range: 20 - 150 or 100 - 550 cm

#endif // SHARP_IRED_PSD_SENSOR_H_INCLUDED
