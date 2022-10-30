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
* [Source file]: "stepper_motor.c" running functions of [ATmega-1284P] and stepping motor control sub-system   *
* ------------------------------------------------------------------------------------------------------------ *
* ------------------------------------------------------------------------------------------------------------ *
* [written by]: Zohrabyan David at Physalis Labs. [Berlin 08.07.2019], Richardstrasse 110, 12043 ------------- *
***************************************************************************************************************/
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//----------------------------------- [ Source File stepping motor ] -----------------------------------------//
//------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------//

#include <avr/io.h>
#include <Hardware_Bay.h>
#include <stepper_motor.h>

//----------------------------- working variables ----------------------------//

extern int8_t stepper_speed;                    // variable controls stepping motor speed, can be changed over PCLink()

volatile int8_t direction = FORWARD;            // sighed variable to control stepping motor rotation direction
volatile uint8_t stepPhase = 0;                 // variable to increment and decrement the motor phases inside array
volatile uint16_t stepCounter = 0;              // simply counts steps taken in either direction

const uint8_t motor_Phases[] = {

    (1 << COIL_B1),                             // full step
    (1 << COIL_B1) | (1 << COIL_A2),            // half step
    (1 << COIL_A2),                             // full step
    (1 << COIL_A2) | (1 << COIL_B2),            // half step
    (1 << COIL_B2),                             // full step
    (1 << COIL_B2) | (1 << COIL_A1),            // etc..
    (1 << COIL_A1),
    (1 << COIL_A1) | (1 << COIL_B1),
};

//---------------------------- functions ----------------------------//

void stepperDrive_adapted(int8_t number_of_steps, uint8_t delay){ // adapted version for standalone usage

    if(number_of_steps > 0){

        direction = FORWARD ;
    }
    else{
        direction = BACKWARD;
        number_of_steps = -number_of_steps;
    }

    OCR0A = delay;                               // delay in counter compare register
    stepCounter = 0;                             // initialize to zero steps taken so far
    TIMSK0 |= (1 << OCIE0A);                     // turn on interrupts! stepping...
    while(!(stepCounter == number_of_steps)){;   // wait ...
    }
    TIMSK0 &= ~(1 << OCIE0A);                    // turn-off interrupts
}

void stepperDrive(uint8_t number_of_steps, uint8_t delay){ //normal version, used inside the trapezoidDrive_Stepper()

    OCR0A = delay;                               // delay in counter compare register
    stepCounter = 0;                             // initialize to zero steps taken so far
    TIMSK0 |= (1 << OCIE0A);                     // turn on interrupts! stepping...
    while(!(stepCounter == number_of_steps)){;   // wait ...
    }
    TIMSK0 &= ~(1 << OCIE0A);                    // turn-off interrupts
}

void de_EnergizeStepper(void){

    STEPPER_PORT &= ~(1 << COIL_A1);
    STEPPER_PORT &= ~(1 << COIL_A2);
    HALL_PORT &= ~(1 << COIL_B1);
    HALL_PORT &= ~(1 << COIL_B2);
}

void trapezoidDrive_Stepper(int8_t number_of_steps){

    uint8_t delay = MAX_DELAY;
    uint16_t stepsTaken = 0;

    if(number_of_steps > 0){

        direction = FORWARD;
    }
    else{
        direction = BACKWARD;
        number_of_steps = -number_of_steps;
    }

    if(number_of_steps > (RAMP_STEPS * 2)){                         // have enough steps for a full trapezoid move

        while(stepsTaken < RAMP_STEPS){                             // accelerate
                stepperDrive(1, delay);                             // function call
                delay -= ACCELERATION;
                stepsTaken ++;
        }                                                           // cruise

        delay = MIN_DELAY;
        stepperDrive((number_of_steps - 2 * RAMP_STEPS), delay);
        stepsTaken += (number_of_steps - 2 * RAMP_STEPS);
                                                                    // decelerate
        while(stepsTaken < number_of_steps){
                stepperDrive(1, delay);
                delay += ACCELERATION;
                stepsTaken ++;
        }
    }
    else{                                                          // partial ramp-up or down
        while(stepsTaken <= number_of_steps / 2){
            stepperDrive(1, delay);
            delay -= ACCELERATION;
            stepsTaken ++;
        }
        delay += ACCELERATION;
        while(stepsTaken < number_of_steps){
            stepperDrive(1, delay);
            delay += ACCELERATION;
            stepsTaken ++;
        }
    }
}
