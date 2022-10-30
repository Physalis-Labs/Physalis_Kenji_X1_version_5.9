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
* [Header file]: "stepping_motor.h" defining all functionality                                                 *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* [written by]: Zohrabyan David at Physalis Labs. [Berlin 08.07.2019], Richardstrasse 110, 12043 ------------- *
***************************************************************************************************************/
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//----------------------------------- [ Header File stepping motor ] -----------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//

#ifndef STEPPER_MOTOR_H_INCLUDED
#define STEPPER_MOTOR_H_INCLUDED
                                                                                                                        // stepper motor driving parameter
#define FORWARD            2                                                              // Set these to +/- 1 for half-stepping, +/- 2 for full-stepping
#define BACKWARD          -2
#define TURN              200                                                   // 360� / 1.8� = 200 steps per rotation for VEXTA PH-265L, motor dependent

#define MAX_DELAY         255                                                                                              // determines min startup speed
#define MIN_DELAY         255                                                                                               // determines max cruise speed
#define ACCELERATION      150                                                                                // lower = smoother, but slower acceleration

#define RAMP_STEPS        (MAX_DELAY - MIN_DELAY) / ACCELERATION


void stepperDrive_adapted(int8_t number_of_steps, uint8_t delay);   // adapted version of stepperDrive() standalone, energizes stepper-motor and drives it

void stepperDrive(uint8_t number_of_steps, uint8_t delay);                                        // stepperDrive(), used inside trapezoid Drive_Stepper()

void trapezoidDrive_Stepper(int8_t number_of_steps);                                               // energizes step-motor drives it with accelerated move

void de_EnergizeStepper();                                                // de-energizes stepper-motor coils, does not hold the position, torque release!

#endif // STEPPER_MOTOR_H_INCLUDED
