/*****************************************************************************************************************************
* ------------------------------------------------- =( Physalis Labs. )= --------------------------------------------------- *
* ---------------------------- =( Kenji-X1 Tele-Presence UGV Platform Firmware Source Code )= ------------------------------ *
* -------------------------------------------------------------------------------------------------------------------------- *
* --------------------------------------------- =( Firmware Version 5.8 )= ------------------------------------------------- *
* ----------------------------- =( Automated UGV for Tele-Presence and Remote Operations )= -------------------------------- *
* -------------------------------------------------------------------------------------------------------------------------- *
* -------------------------------------------------------------------------------------------------------------------------- *
* ---- [ Electronic Systems and Hardware Technical Specifications ] -------------------------------------------------------- *
* -------------------------------------------------------------------------------------------------------------------------- *
* [MCU/ Micro-Controller]: Atmel ATmega-1284p picoPower AVR 8-Bit RISC Architecture CPU Frequency = 20 MHz                   *
* [Drivetrain / Power Unit]: Dedicated H-Bridges with Dual (25 mm) Geared DC Motors, Reduction Factor 1:150                  *
* [Drivetrain / Stall Current]: 4500 mA                                                                                      *
* [Drivetrain / Stall Torque]: 9.5 kg NaN                                                                                    *
* [Drivetrain / Rated Current]: 1200 mA                                                                                      *
* [Drivetrain / Noise]: 56 dB                                                                                                *
* [Drivetrain / Working Voltage]: 9 Volt DC                                                                                  *
* [Drivetrain Tower/MastCam ]: Stepper-Online 1.8-Angle 200 Steps, 2-Phase (12V, 500mA), Stepping Motor on Planetary Gearbox *
* [Photocells]: Advanced Photonix/NSL-5910, PHOTOCELL, CDS TO-8 Hermetic, Photo-Conductive Cells (Cadmium-Sulfide)           *
* [Vibration Detection Unit]: Murata Electronics/7BB-27-4CL0, Vibration Measurement Probe on Piezo-Electric Sensor           *
* [Infrared Proximity Sensor]: Sharp GP2Y0A02YK0F (20 - 150 cm op. range), (PSD + IRED), Analog                              *
* [Speaker]: Visaton, 5V, DC, Impedance = 8 Ohm, (System Alarms and Sounds)                                                  *
* [Bar-Graph]: Broadcom Limited/HDSP-483210, Green-Yellow-Red, 10-Seg., (Indications of Various System States)               *
* [Lights]: Cree High-Power LED, on Convoy Driver PCB, TIR Lens, Warm White                                                  *
* [WLAN Module]: SBC (RaspberryPi B+/Zero-W), Wireless Communication Unit and Remote-Control Functionality                   *
* [Battery Unit]: Panasonic NCR18650B 3.7 v 3400 mAh, Li-Polymer Cells Assembled 3 x 3.7V 3500mA = 12.700V                   *
* [EEPROM]: Microchip 256 kByte EEPROM, non-Volatile Memory and System Critical Data Storage                                 *
* [LIDAR]: Garmin LIDAR-Lite v3HP, Update_rate >= 1KHz, Res = 1cm, Accur = -/+2.5cm, IPX7-Rated, Range = 5cm-40m             *
* [FRAM]: Cypress Semiconductor FM25V20A-G-ND, 2-Mbit Ferroelectric RAM (automotive), non-Volatile Memory, and Data Storage  *
* [GPIO-I2C-Expander]: NXP/PCF8575TS, 16-Bit GPIO Expander on I2C BUS, adds 16 GPIOs to ATmega-1284p MCU                     *
* [Display]: 16 x 2 Character Display on Motorola HD44780 1602 LCD Driver IC, on I2C BUS (Color = Blue/White)                *
* [PWM Controller]: NXP/PCA9685PW, 16-GPIO, Res=12-Bit, PWM-Controller,I2C-BUS,Controls Mecha-Arms Servo Drives              *
* [Digial Temperature Sensor]: NXP/LM75ADP-118, Temperature Sensor, Res=0.125°C, I2C-BUS, Sigma-Delta A-D Conv.              *
* [Mecha-Arms]: DSSERVO (RDS5160,3216,3115MG), High-Torque, Servo-Motors (Left/Right)]: 7 x Servos per Arm and 2 x Claws     *
* [USB-to-UART Bridge]: Cypress Semiconductor CY7C65213-28PVXIT, USB 2.0 Full-Speed controller, and a UART-Transceiver       *
* -------------------------------------------------------------------------------------------------------------------------- *
* [Source File]: "main.c" Running all Input/Output Operations of [ATmega-1284p] MCU ---------------------------------------- *
* -------------------------------------------------------------------------------------------------------------------------- *
* -------------------------------------------------------------------------------------------------------------------------- *
* [written by]:Dr.-Ing. Zohrabyan David, Physalis Labs. [Potsdam 26.10.2022], Hans-Marchwitza-Ring.21, 14473 --------------- *
*****************************************************************************************************************************/
//--------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------- [ Source File ] -----------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------- [Preamble] ---------------------------------------------------------------------//
#include <avr/io.h>
#include <Hardware_Bay.h>
#define __DELAY_BACKWARD_COMPATIBLE__                       // used for _delay_ms() function to call with variable as argument
#include <util/delay.h>
#include <USART.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <scale16.h>
#include <stepper_motor.h>
#include <avr/sleep.h>
#include <battery_pack.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <sharp_IRED_PSD_sensor.h>
#include <vibration_sensor.h>
//#include <Microchip_25LC256.h>                                          // currently not in use, replaced with Cypress FRAM
#include <FM25V10.h>                                                      // Cypress FRAM is used!
#include <i2c.h>
#include <util/twi.h>
#include <lcd_display.h>
#include <PCF_8575.h>
#include <Garmin_LIDAR-Lite_v3HP.h>
//#include <PCA9685.h>
#include <PCA9685_PWM_Controller.h>
#include <reportUART_12bytes.h>

#define OVERFLOWS_PER_SECOND        1220                      // (F_CPU = 20 MHz, F_OSC Timer / 64) / 255 = ~ 1220 overflows
#define OVERFLOWS_PER_MILLISECOND   2                   // (F_CPU = 20 MHz, F_OSC Timer / 64) /255 / 1000 = ~ 1.22 overflows
#define NOTE_DURATION               0xF000                                                    // determines long note length
// --------------------- I-2C ------------------------------------------------------------------------------------------- //
//#define LCD_I2C_ADDR                0b01001111
#define LM75_ADDRESS_W              0b10010000
#define LM75_ADDRESS_R              0b10010001
#define LM75_TEMP_REGISTER          0b00000000
#define LM75_CONFIG_REGISTER        0b00000001
#define LM75_THYST_REGISTER         0b00000010
#define LM75_TOS_REGISTER           0b00000011
// --------------------- I-2C ------------------------------------------------------------------------------------------ //
#define CURRENT_LOCATION_POINTER    0                          // where to store a pointer to the current reading in EEPROM
#define SECONDS_POINTER             2                                                     // store seconds-delay value here
#define MEMORY_START                4                                          // where to start logging temperature values
#define MENU_DELAY                  5                                         // seconds to wait before bypassing main menu
// --------------------- FRAM Addresses for Mech.-arm Servo States  ---------------------------------------------------- //
#define SERVOS_POSITION_ADDRESS     65500                                                   // servo positions FRAM address
#define PARKING_STATE_ADDRESS       65600                                                 // arm parking state FRAM address

extern char move_direction[];                                              // variable which holds the robo-Drive direction
extern int8_t direction;
extern uint8_t motor_Phases[];
extern uint8_t stepPhase;
extern uint16_t stepCounter;

int8_t  stepper_speed = 5;                         // holds step motor speed, possible to change on the fly via getnumber()
uint8_t drive_speed = 20;                                                   //roboDrive() variables to pass value from UART
uint8_t servo_speed_mecha_arms = 20;                    // holds mecha-arms servos speed, change on the fly via getnumber()
uint8_t servo_speed_turning_mecha_arms = 20;           // holds mecha-arms turning(L/R) servo speed, change via getnumber()
uint8_t servo_speed_towerdrive = 20;                 // holds tower servo vertical speed, change on the fly via getnumber()
uint8_t servo_speed_grippers = 20;                   // both grippers speed (left/right), change on the fly via getnumber()
uint8_t servo_speed_rotation_grippers = 20;           // both grippers rotation servo speed before updating via getnumber()

volatile bool move_knob_R, move_knob_L;                            // variables tracking movement direction and speed right
volatile bool move_dir_R, move_dir_L;                               // variables tracking movement direction and speed left

volatile short hall_S2R_pulse;                                          // variables for counting Hall sensor pulses /right
volatile short hall_S2L_pulse;                                           // variables for counting Hall sensor pulses /left

volatile uint16_t year_ = 2022;                                                           // variable keeps track of years
volatile uint8_t months_ = 11;                                                            // variable keeps track of months
volatile uint8_t days_ = 5;                                                               // variable keeps track of days
volatile uint8_t hours_ = 12;                                                             // variable keeps track of hours
volatile uint8_t minutes_ = 30;                                                         // variable keeps track of minutes
volatile uint8_t seconds_ = 25;                                                         // variable keeps track of seconds
volatile uint16_t milliseconds_ = 56;                                              // variable keeps track of milliseconds
volatile uint16_t sys_tick = 0;                                                       // system tick to derive a time-base

uint16_t ambient_light;                                                                           // photo-conductive cell
uint16_t ired_range, scanner_minVal;                                               // analog data from IRED sensor (sharp)

uint16_t currentNoteLength = NOTE_DURATION / 2;                                                     // current note length

const uint8_t keys[] = {'a', 'w', 's', 'e', 'd', 'f', 't', 'g', 'y', 'h', 'j', 'i', 'k', 'o', 'l', 'p', ';', '\''};
const uint16_t notes[] = {G4, Gx4, A4, Ax4, B4, C5, Cx5, D5, Dx5, E5, F5, Fx5, G5, Gx5, A5, Ax5, B5, C6};

extern float power_pack_voltage;                                           // the voltage of the power supply on the robot
char voltage_buffer[20];
extern uint32_t theta_0, theta_1;

float distance;                                                                         // variable holding distance in cm
char distance_buffer[20];                                                                  // string buffer, for sprintf()
char user_input;                                                                             // takes user input character
uint16_t test_cycle_num = 0;                                                                 // number of cycles for tests
uint32_t i;                                                                                     // counter for cycles loop

uint16_t secondsDelay;                                                                // how long to wait between readings
uint32_t currentMemoryLocation;                                                                   // where are we in FRAM?
uint16_t i_;                                                                         // used for counting memory locations
uint8_t address;                                                                        // variable holding FRAM addresses
uint8_t enterMenu;                                                                            // logical flag, logger menu
uint8_t tempHighByte, tempLowByte, temperatureByte;                                               // data bytes from LM-75
char temp_buffer[20];                                                                      // string buffer, for sprintf()

uint8_t LIDAR_status = 0;                                                           // Garmin LIDAR status / busy/free 1/0
uint16_t LIDAR_distance = 0;                                                                             // distance in cm
char LIDAR_range_buffer[10];                                                               // string buffer, for sprintf()

bool gripper_State = 1;
//------------------------ Initialization Functions --------------------------------------------------------------------//
void initUSART(void){

  UBRR0H = 0b00000000;                            // BAUD rate value for 0.5 Mbit,  high-byte   UBRRn value from datasheet
  UBRR0L = 0b00000100;                            // BAUD rate value for 0.5 Mbit,  low-byte    UBRRn value from datasheet
  UCSR0A |= (1 << U2X0);                                                                              // double speed mode
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);                                               // Enable USART transmitter/receiver
  //UCSR0B |= (1 << RXCIE0);                                                             // Enable USART Receive Interrupt
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                                                       // 8 data bits, 1 stop bit
}

static inline void initGPIOchangeInterrupts(void){

    PCICR |= (1 << PCIE2);                                   // enable GPIO state change interrupts on BANK-C GPIO(31-24)
    PCMSK2 |= (1 << PCINT18);// | (1 << PCINT20);             // enable specific interrupt on GPIO-24/PC2 and GPIO-26/PC4
}

static inline void initTimer_0(void){

    TCCR0A |= (1 << WGM01);                                                                                   // CTC mode
    TCCR0B |= (1 << CS00) | (1 << CS02);                                          // clock prescaler set F_CPU / div 1024
    OCR0A = MAX_DELAY;                                                                          // set default slow speed
}

static inline void initTimer_1(void){

    TCCR1A |= (1 << WGM10);                                                               // Phase Correct PWM 8-Bit Mode
    TCCR1B |= ((1 << CS11) | (1 << CS10));                                                        // prescaler = F_CPU/64
    TCCR1A |= ((1 << COM1A1) | (1 << COM1B1));                                       // toggle OC1A/OC1B on compare match
    TIMSK1 |= (1 << TOIE1);                                                          // Timer-1 overflow interrupt enable
}

static inline void initTimer_2(void){

    TCCR2A |= (1 << WGM20);                                                               // Phase Correct PWM 8-Bit Mode
    TCCR2B |= (1 << CS22);                                                                        // prescaler = F_CPU/64
    TCCR2A |= ((1 << COM2A1) | (1 << COM2B1));                                       // toggle OC2A/OC2B on compare match
}

static inline void initTimer_3(void){
                                                                                                 // Normal Mode, counting
    TCCR3B |= (1 << CS30);                                                                        // prescaler div / 1024
}                                                                                          // hardware pins not connected

static inline void initADC(void){

    ADMUX |= (1 << REFS0);                       // ADC Voltage Reference Selection: AVCC with external capacitor on AREF
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);                                  // ADC clock prescaler set div./128 ~ 313 kHz
    ADCSRA |= (1 << ADEN);                                                     // enable on-board ADC hardware peripheral
    //ADCSRA |= (1 << ADIE);                                                 // ADC conversion complete interrupt enabled
}

//------------------------ Kinematics Functions -----------------------------------------------------------------------//
static inline void setRightBridgeState(uint8_t right_sub_L, uint8_t right_sub_R, uint8_t pwm_pulse_R){
                                                                              // configures the state of the right bridge
    if(right_sub_L){
        OCR1A = pwm_pulse_R;
        CTRL_PORT |= (1 << RBN3_FET);
        CTRLPWM_DDR |= (1 << RBN1_FET);
    }
    else{
        OCR1A = pwm_pulse_R;
        CTRL_PORT &= ~(1 << RBN3_FET);
        CTRLPWM_DDR &= ~(1 << RBN1_FET);
    }
    if(right_sub_R){
        OCR1B = pwm_pulse_R;
        CTRL_PORT |= (1 << RBN4_FET);
        CTRLPWM_DDR |= (1 << RBN2_FET);
    }
    else{
        OCR1B = pwm_pulse_R;
        CTRL_PORT &= ~(1 << RBN4_FET);
        CTRLPWM_DDR &= ~(1 << RBN2_FET);
    }
}

static inline void setLeftBridgeState(uint8_t left_sub_L, uint8_t left_sub_R, uint8_t pwm_pulse_L){
                                                                              // configures the state of the left bridge
    if(left_sub_L){
        OCR2A = pwm_pulse_L;
        CTRL_PORT |= (1 << LBN7_FET);
        CTRLPWM_DDR |= (1 << LBN5_FET);
    }
    else{
        OCR2A = pwm_pulse_L;
        CTRL_PORT &= ~(1 << LBN7_FET);
        CTRLPWM_DDR &= ~(1 << LBN5_FET);
    }
    if(left_sub_R){
        OCR2B = pwm_pulse_L;
        CTRL_PORT |= (1 << LBN8_FET);
        CTRLPWM_DDR |= (1 << LBN6_FET);
    }
    else{
        OCR2B = pwm_pulse_L;
        CTRL_PORT &= ~(1 << LBN8_FET);
        CTRLPWM_DDR &= ~(1 << LBN6_FET);
    }
}

void roboDrive(char move_direction[], uint8_t speed){        // controls motors, ground speed aka propulsion sub-system

    if(strcmp(move_direction, "forward") == 0){
        setLeftBridgeState(1, 0, speed);
        setRightBridgeState(1, 0, speed);
        //printString("robo-drive: FORWARD at speed: ");
        //printByte(speed);
//        printString("\r\n");
    }
    else if(strcmp(move_direction, "right") == 0){
        setLeftBridgeState(1, 0, speed);
        setRightBridgeState(0, 1, speed);
//        printString("robo-drive: LEFT at speed: ");
//        printByte(speed);
//        printString("\r\n");
    }
    else if(strcmp(move_direction, "left") == 0){
        setLeftBridgeState(0, 1, speed);
        setRightBridgeState(1, 0, speed);
//        printString("robo-drive: RIGHT at speed: ");
//        printByte(speed);
//        printString("\r\n");
    }
    else if(strcmp(move_direction, "backward") == 0){
        setLeftBridgeState(0, 1, speed);
        setRightBridgeState(0, 1, speed);
//        printString("robo-drive: BACKWARD at speed: ");
//        printByte(speed);
//        printString("\r\n");
    }
    else if(strcmp(move_direction, "brake") == 0){
        setLeftBridgeState(0, 0, 0);
        setRightBridgeState(0, 0, 0);
//        printString("robo-drive: BRAKE at speed: ");
//        printByte(speed);
//        printString("\r\n");
    }
    else{
        printString("robo-drive: What? ...");
        printString("\r\n");
    }
}

 uint16_t find_Min(uint16_t *array, uint16_t size){                      // finds the minimum value in array of data

     uint16_t min = array[0], i;

     for(i = 1; i < size; i ++){
        if(array[i] < min){
            min = array[i];
        }
     }
     return(min);
 }

 void move_Servo_Bidirect(uint8_t servo_number, int8_t servo_new_angle, uint8_t servo_angular_speed, uint8_t servo_acceleration){

    int8_t servo_step;
    int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
    if((servo_number > 15) || (servo_number < 0)) {
        printString("\r\nmove_Servo_Bidirect: invalid arguments ¯\\(°_o)/¯");
    }
    else{
    if(servo_current_angle >= 0 && servo_new_angle >= 0){
            if(servo_current_angle < servo_new_angle){
                for(servo_step = servo_current_angle; servo_step <= servo_new_angle ; servo_step += servo_acceleration){
                runServo_onPCA9685(servo_number, servo_step);
                FRAM_writeByte(SERVOS_POSITION_ADDRESS + servo_number, servo_new_angle);
                _delay_ms(servo_angular_speed);
                }
            }
            else{
                for(servo_step = servo_current_angle; servo_step >= servo_new_angle ; servo_step -= servo_acceleration){
                runServo_onPCA9685(servo_number, servo_step);
                FRAM_writeByte(SERVOS_POSITION_ADDRESS + servo_number, servo_new_angle);
                _delay_ms(servo_angular_speed);
                }
            }
    }
    else if(servo_current_angle >= 0 && servo_new_angle <= 0){
                for(servo_step = servo_current_angle; servo_step >= servo_new_angle ; servo_step -= servo_acceleration){
                runServo_onPCA9685(servo_number, servo_step);
                FRAM_writeByte(SERVOS_POSITION_ADDRESS + servo_number, servo_new_angle);
                _delay_ms(servo_angular_speed);
            }
    }
    else if(servo_current_angle <= 0 && servo_new_angle >= 0){
                for(servo_step = servo_current_angle; servo_step <= servo_new_angle ; servo_step += servo_acceleration){
                runServo_onPCA9685(servo_number, servo_step);
                FRAM_writeByte(SERVOS_POSITION_ADDRESS + servo_number, servo_new_angle);
                _delay_ms(servo_angular_speed);
            }
    }
    else if(servo_current_angle <= 0 && servo_new_angle <= 0){
            if(servo_current_angle < servo_new_angle){
                for(servo_step = servo_current_angle; servo_step <= servo_new_angle ; servo_step += servo_acceleration){
                runServo_onPCA9685(servo_number, servo_step);
                FRAM_writeByte(SERVOS_POSITION_ADDRESS + servo_number, servo_new_angle);
                _delay_ms(servo_angular_speed);
                }
            }
            else{
                for(servo_step = servo_current_angle; servo_step >= servo_new_angle ; servo_step -= servo_acceleration){
                runServo_onPCA9685(servo_number, servo_step);
                FRAM_writeByte(SERVOS_POSITION_ADDRESS + servo_number, servo_new_angle);
                _delay_ms(servo_angular_speed);
                }
            }
    }
    else{}

    printString("\r\n roboDrive >> Servo ----------->");
    printByte(servo_number);
    printString("<--- is moved.");

    for(int i = 0; i <= 15; i ++ ){
      printString("\r\n FRAM at addr ");
      printWord(SERVOS_POSITION_ADDRESS + i);
      printString(" stores ");
      printByte(FRAM_readByte(SERVOS_POSITION_ADDRESS + i));

      if (i == servo_number){
        printString(" <--- new angle");
        }
    }
  }
}
// ----------------------------------- retired Mecha-Arms Control functions (use updated ones below) ------------------------//
// void roboDrive_Arm_Stretch_clockwise(int8_t stretch_angle, uint8_t engaged_servos, uint8_t combined_angular_speed){
//
//     int8_t current_angle_arm, i;
//     if(stretch_angle > 0){
//         for(i = 0; i <= engaged_servos; i ++){
//                 move_Servo_Bidirect(i, stretch_angle, combined_angular_speed, 1);
//     }
// }
//     else{
//         printString("\r\n roboDrive >> invalid stretch argument ¯\\(°_o)/¯ \r\n");
//     }
//     current_angle_arm = stretch_angle;
//     printString("\r\n roboDrive >> Arm is Stretched(clockwise)\r\n");
// }
//
// void roboDrive_Arm_Stretch_anticlockwise(int8_t stretch_angle, uint8_t engaged_servos, uint8_t combined_angular_speed){
//
//     int8_t current_angle_arm, i;
//     if(stretch_angle < 0){
//         for(i = 0; i <= engaged_servos; i ++){
//                 move_Servo_Bidirect(i, stretch_angle, combined_angular_speed, 1);
//     }
// }
//     else{
//         printString("\r\n roboDrive >> invalid stretch argument ¯\\(°_o)/¯ \r\n");
//     }
//     current_angle_arm = stretch_angle;
//     printString("\r\n roboDrive >> Arm is Stretched(anti-clockwise)\r\n");
// }
//
// void roboDrive_Arm_Reset(int8_t servo_number){
//
//     if((servo_number > 15) || (servo_number < 0)) {
//         printString("\r\n roboDrive_Arm_Reset: invalid servo number ¯\\(°_o)/¯");
//     }
//     else{
//         if(FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number) != 0) {
//             move_Servo_Bidirect(servo_number, 0, 30, 1);
//             printString("\r\n roboDrive >> Servo ");
//             printByte(servo_number);
//             printString(" is reset(rdAR). Delay.\r\n");
//             _delay_ms(500);
//         }
//         else{
//             printString("\r\n roboDrive >> Servo ");
//             printByte(servo_number);
//             printString(" was already 0, no need to reset(rdAR).\r\n");
//         }
//     }
// }
//
// void roboDrive_Arm_Global_Reset(void){
//
//     uint16_t reset_time_theta = 1000;
//     printString("\r\n roboDrive >> Executing full arm reset.\r\n");
//     for(int i = 0; i <= 15; i ++){
//             roboDrive_Arm_Reset(i);
//     }
//     FRAM_writeByte(PARKING_STATE_ADDRESS, 0);                                               //write parking mode to FRAM
//     printString("\r\n roboDrive >> Arm is Reset!\r\n");
//     printString("\r\n roboDrive >> Disengaging ...\r\n");
// }
//
// void roboDrive_Claw_open(uint8_t clamp_force, int8_t clamp_angle){
//
//             move_Servo_Bidirect(6, clamp_angle, clamp_force, 1);
//             printString("\r\n roboDrive >> Claw opened...\r\n");
// }
//
// void roboDrive_Claw_close(int8_t clamp_angle, uint8_t clamp_force, uint8_t clamp_acceleration){
//
//             move_Servo_Bidirect(6, clamp_angle, clamp_force, clamp_acceleration);
//             printString("\r\n roboDrive >> Claw closed...\r\n");
// }
//
// void roboDrive_Claw_turn(int8_t claw_turn_target_angle, uint8_t claw_angular_speed, uint8_t claw_rotation_acceleration){
//
//             move_Servo_Bidirect(5, claw_turn_target_angle, claw_angular_speed, claw_rotation_acceleration);
//             printString("\r\n roboDrive >> Claw turned.\r\n");
// }
//
// void roboDrive_Arm_Park(void){
//
//             int8_t parking_state = FRAM_readByte(PARKING_STATE_ADDRESS);
//             printString("\r\nParking state is ");
//             printByte(parking_state);
//             printString(".\r\n roboDrive >> Choose mode: \r\n1 <-- Park [default arm park mode] UnPark --> 2");
//             printString("\r\n3 <-- Park [alternate arm park mode] UnPark --> 4\r\n");
//             switch (receiveByte()) {
//                     case '1':
//                             if((parking_state == 1) || (parking_state == 2)){
//                                 printString("\r\n roboDrive >> Its unwise to park again when you already are parked. \r\n");
//                             }
//                             else{
//                                 move_Servo_Bidirect(4, -90, 30, 1);                       // Right Arm, parking commands
//                                 move_Servo_Bidirect(2, -90, 30, 1);
//                                 move_Servo_Bidirect(4, 90, 30, 1);
//                                 move_Servo_Bidirect(1, -90, 40, 1);
//                                 move_Servo_Bidirect(3, 80, 30, 1);
//                                 move_Servo_Bidirect(1, -110, 40, 1);
//
//                                 // move_Servo_Bidirect(11, 90, 30, 1);                     // Left Arm, parking commands
//                                 // move_Servo_Bidirect(9,  90, 30, 1);
//                                 // move_Servo_Bidirect(11, -90, 30, 1);
//                                 // move_Servo_Bidirect(8,  -90, 40, 1);
//                                 // move_Servo_Bidirect(10, -90, 30, 1);
//                                 // move_Servo_Bidirect(8, -110, 40, 1);
//
//                                 FRAM_writeByte(PARKING_STATE_ADDRESS, 1);                   //write parking mode to FRAM
//                                 printString("\r\n roboDrive >> Arm Parked. Parking mode: 1, default.\r\n");
//                             }
//                         break;
//                     case '2':
//                             if(parking_state != 1){
//                                 printString("\r\n roboDrive >> Its unwise to run un-park 1 from any state but parked mode 1. \r\n");
//                             }
//                             else{
//                               move_Servo_Bidirect(1, -70, 40, 1);                         // Right Arm, parking commands
//                               move_Servo_Bidirect(4, -50, 30, 1);
//                               move_Servo_Bidirect(2, -40, 30, 1);
//
//                               move_Servo_Bidirect(2, 0, 30, 1);
//                               move_Servo_Bidirect(4, 0, 30, 1);
//                               move_Servo_Bidirect(3, 0, 30, 1);
//
//                               // move_Servo_Bidirect(8, 70, 40, 0);                        // Left Arm, parking commands
//                               // move_Servo_Bidirect(11, 50, 30, 0);
//                               // move_Servo_Bidirect(9, 40, 30, 1);
//                               //
//                               // move_Servo_Bidirect(9, 0, 40, 0);
//                               // move_Servo_Bidirect(11, 0, 30, 0);
//                               // move_Servo_Bidirect(10, 0, 40, 1);
//
//                                 FRAM_writeByte(PARKING_STATE_ADDRESS, 0);                  // write parking mode to FRAM
//                                 printString("\r\n roboDrive >> Arm UnParked from mode 1.\r\n");
//                             }
//                         break;
//                     case '3':
//                             if((parking_state == 1) || (parking_state == 2)){
//                                 printString("\r\n roboDrive >> Its unwise to park again when you already are parked. \r\n");
//                             }
//                             else{
//                                 move_Servo_Bidirect(4, 90, 30, 1);
//                                 move_Servo_Bidirect(3, 90, 30, 1);
//                                 move_Servo_Bidirect(0, -70, 30, 1);
//                                 FRAM_writeByte(PARKING_STATE_ADDRESS, 2);                  //write parking mode to FRAM
//                                 printString("\r\n roboDrive >> Arm Parked. Parking mode: 2, alternate.\r\n");
//                             }
//                         break;
//                     case '4':
//                             if(parking_state != 2){
//                                 printString("\r\n roboDrive >> Its unwise to run un-park 2 from any state but parked mode 2. \r\n");
//                             }
//                             else{
//                                 roboDrive_Arm_Global_Reset();
//                                 printString("\r\n roboDrive >> Arm UnParked from mode 2. Kinda.\r\n");
//                             }
//                         break;
//                     default:
//                         printString("\r\n roboDrive >> Arm parking mode unrecognized ¯\\(°_o)/¯\r\n");
//             }
// }

void roboDrive_Arm_Testing(void){

            uint8_t servo_number = 100;     // defaults set to unsafe values, should be filtered by move_Servo_Bidirect
            int8_t servo_new_angle = 0;      // defaults set to unsafe values, should be filtered by runServo_onPCA9685
            uint8_t servo_angular_speed = 40;
            uint8_t servo_acceleration = 1;

            uint8_t angle_user_input = 0;
            uint8_t user_input_filter = 0;                                              // filter for extra "Enter" hit

            printString("\r\n>> custom mecha-arm movement parameters:");
            printString("\r\nenter servo Nr >> [0-14]: ");
            servo_number = getNumber();
            printString("\r\nenter target angle [0-125]: ");
            angle_user_input = getNumber();
            printString("\r\nenter direction [+ or 0 for anticlockwise, - or 1 for clockwise]: \r\n");
            switch (receiveByte()) {                                                                     // take input
                    case '+':
                        servo_new_angle = (int8_t) angle_user_input;
                        printString("\t\t positive, anticlockwise");
                        break;
                    case '0':
                        servo_new_angle = (int8_t) angle_user_input;
                        printString("\t\t positive, anticlockwise");
                        break;
                    case '-':
                        servo_new_angle = (int8_t) angle_user_input;
                        servo_new_angle = (-1) * servo_new_angle;
                        printString("\t\t negative, clockwise");
                        break;
                    case '1':
                        servo_new_angle = (int8_t) angle_user_input;
                        servo_new_angle = (-1) * servo_new_angle;
                        printString("\t\t negative, clockwise");
                        break;
                    default:
                        printString("\t\t direction unrecognized, abort by setting servo ID to 100.");
                        servo_number = 100;
            }
            user_input_filter = receiveByte();
            printString("\r\nenter servo speed >> [0-255]: ");
            servo_angular_speed = getNumber();
            printString("\r\nenter servo acceleration >> [0-255]: ");
            servo_acceleration = getNumber();
            move_Servo_Bidirect(servo_number, servo_new_angle, servo_angular_speed, servo_acceleration);
            printString("\r\nmove_Servo_Bidirect executed.");
}

// ----------------------------------- updated Mecha-Arms Control functions D.Z. --------------------------------------------//
void roboDrive_RecoverArmsForParking(uint8_t recover_speed){
                                       // apply only when both arms are streched, use to recover from unknown states to parking
// ------------------------------------------------- // right mecha arm op-codes, // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(0,  0,  recover_speed, 1);
  move_Servo_Bidirect(4,  0,  recover_speed, 1);
  move_Servo_Bidirect(3,  0,  recover_speed, 1);
  move_Servo_Bidirect(2,  0,  recover_speed, 1);
  move_Servo_Bidirect(1,  0,  recover_speed, 1);
  move_Servo_Bidirect(5,  0,  recover_speed, 1);
  move_Servo_Bidirect(6, 30,  recover_speed, 1);
// ------------------------------------------------------------------------------------------------ // left mecha-arm op-codes
  move_Servo_Bidirect(7,    0,  recover_speed, 1);
  move_Servo_Bidirect(11,   0,  recover_speed, 1);
  move_Servo_Bidirect(10,   0,  recover_speed, 1);
  move_Servo_Bidirect(9,    0,  recover_speed, 1);
  move_Servo_Bidirect(8,    0,  recover_speed, 1);
  move_Servo_Bidirect(12,   0,  recover_speed, 1);
  move_Servo_Bidirect(13, -20,  recover_speed, 1);
}

void roboDrive_RecoverLeftArmForParking(uint8_t recover_speed){
                                              // apply only when arm is streched, use to recover from unknown states to parking
// -------------------------------------------------- // Left mecha arm op-codes, // speed >> ((superfast) 1 - 255 (very slow))
// ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------- // left mecha-arm op-codes
  move_Servo_Bidirect(7,    0,  recover_speed, 1);
  move_Servo_Bidirect(11,   0,  recover_speed, 1);
  move_Servo_Bidirect(10,   0,  recover_speed, 1);
  move_Servo_Bidirect(9,    0,  recover_speed, 1);
  move_Servo_Bidirect(8,    0,  recover_speed, 1);
  move_Servo_Bidirect(12,   0,  recover_speed, 1);
  move_Servo_Bidirect(13, -20,  recover_speed, 1);
}

void roboDrive_RecoverRightArmForParking(uint8_t recover_speed){
                                              // apply only when arm is streched, use to recover from unknown states to parking
// ------------------------------------------------- // right mecha arm op-codes, // speed >> ((superfast) 1 - 255 (very slow))
// ------------------------------------------------------------------------------------------------------------------------- //
  move_Servo_Bidirect(0,  0,  recover_speed, 1);
  move_Servo_Bidirect(4,  0,  recover_speed, 1);
  move_Servo_Bidirect(3,  0,  recover_speed, 1);
  move_Servo_Bidirect(2,  0,  recover_speed, 1);
  move_Servo_Bidirect(1,  0,  recover_speed, 1);
  move_Servo_Bidirect(5,  0,  recover_speed, 1);
  move_Servo_Bidirect(6, 30,  recover_speed, 1);
}

void roboDrive_ParkArms(uint8_t parking_speed){
                                                     // ---------- park arms (lef/right) >> parked state (current draw ~ 0.6A)
// ------------------------------------------------- // right mecha arm op-codes // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(6, 30,    parking_speed, 1);
  move_Servo_Bidirect(5,  0,    parking_speed, 1);

  move_Servo_Bidirect(0, -10,   parking_speed, 1);
  move_Servo_Bidirect(2, -78,   parking_speed, 1);
  move_Servo_Bidirect(4, 111,   parking_speed, 1);
  move_Servo_Bidirect(3,  53,   parking_speed, 1);
  move_Servo_Bidirect(1, -111,  parking_speed, 1);
// ------------------------------------------------------------------------------------------------ // left mecha arm op-codes
  move_Servo_Bidirect(13, -25,  parking_speed, 1);
  move_Servo_Bidirect(12,   0,  parking_speed, 1);

  move_Servo_Bidirect(7,   0,   parking_speed, 1);
  move_Servo_Bidirect(9, -70,   parking_speed, 1);
  move_Servo_Bidirect(11, 123,  parking_speed, 1);
  move_Servo_Bidirect(10,  53,  parking_speed, 1);
  move_Servo_Bidirect(8,  100,  parking_speed, 1);
}

void roboDrive_ParkLeftArm(uint8_t parking_speed){
                                                     // ---------- park Left-MechaArm (left) >> parked state (current draw ~ 0.6A)
// ------------------------------------------------------- // Left-MechaArm op-codes // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(13, -25,  parking_speed, 1);
  move_Servo_Bidirect(12,   0,  parking_speed, 1);

  move_Servo_Bidirect(7,   0,   parking_speed, 1);
  move_Servo_Bidirect(9, -70,   parking_speed, 1);
  move_Servo_Bidirect(11, 123,  parking_speed, 1);
  move_Servo_Bidirect(10,  53,  parking_speed, 1);
  move_Servo_Bidirect(8,  100,  parking_speed, 1);
}

void roboDrive_ParkRightArm(uint8_t parking_speed){
                                                     // ---------- park Right-MechaArm (right) >> parked state (current draw ~ 0.6A)
// -------------------------------------------------------- // Right-MechaArm op-codes // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(6, 30,    parking_speed, 1);
  move_Servo_Bidirect(5,  0,    parking_speed, 1);
  move_Servo_Bidirect(0, -10,   parking_speed, 1);
  move_Servo_Bidirect(2, -78,   parking_speed, 1);
  move_Servo_Bidirect(4, 111,   parking_speed, 1);
  move_Servo_Bidirect(3,  53,   parking_speed, 1);
  move_Servo_Bidirect(1, -111,  parking_speed, 1);
}

void roboDrive_DeployArmsState1(uint8_t deploy_speed){
                                          // ------------- deploy arms (state_1) (lef/right) >> state 1 (current draw ~ 0.75A)
// ------------------------------------------------- // right mecha arm op-codes,// speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(0, -10,  deploy_speed, 1);
  move_Servo_Bidirect(2, -78,  deploy_speed, 1);
  move_Servo_Bidirect(4, 111,  deploy_speed, 1);
  move_Servo_Bidirect(3,  53,  deploy_speed, 1);
  move_Servo_Bidirect(1, -90,  deploy_speed, 1);

  move_Servo_Bidirect(5,  0,   deploy_speed, 1);
  move_Servo_Bidirect(6, 30,   deploy_speed, 1);
// ---------------------------------------------------------------------------------------------- //  left mecha arm op-codes
  move_Servo_Bidirect(7,    0, deploy_speed, 1);
  move_Servo_Bidirect(9,  -70, deploy_speed, 1);
  move_Servo_Bidirect(11, 125, deploy_speed, 1);
  move_Servo_Bidirect(10,  53, deploy_speed, 1);
  move_Servo_Bidirect(8,   80, deploy_speed, 1);

  move_Servo_Bidirect(12,   0, deploy_speed, 1);
  move_Servo_Bidirect(13, -20, deploy_speed, 1);
}

void roboDrive_DeployLeft_Arm_State1(uint8_t deploy_speed){
                                          // -------------- deploy left arm (state_1) (left) >> state 1 (current draw ~ 0.75A)
// ------------------------------------------------- // Left mecha arm op-codes, // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(7,    0, deploy_speed, 1);
  move_Servo_Bidirect(9,  -70, deploy_speed, 1);
  move_Servo_Bidirect(11, 125, deploy_speed, 1);
  move_Servo_Bidirect(10,  53, deploy_speed, 1);
  move_Servo_Bidirect(8,   80, deploy_speed, 1);
  move_Servo_Bidirect(12,   0, deploy_speed, 1);
  move_Servo_Bidirect(13, -20, deploy_speed, 1);
}

void roboDrive_DeployRight_Arm_State1(uint8_t deploy_speed){
                                          // ------------ deploy right arm (state_1) (right) >> state 1 (current draw ~ 0.75A)
// ------------------------------------------------ // right mecha arm op-codes, // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(0, -10,  deploy_speed, 1);
  move_Servo_Bidirect(2, -78,  deploy_speed, 1);
  move_Servo_Bidirect(4, 111,  deploy_speed, 1);
  move_Servo_Bidirect(3,  53,  deploy_speed, 1);
  move_Servo_Bidirect(1, -90,  deploy_speed, 1);
  move_Servo_Bidirect(5,  0,   deploy_speed, 1);
  move_Servo_Bidirect(6, 30,   deploy_speed, 1);
}
//----------------working here>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void roboDrive_DeployLeft_Arm_State4(uint8_t deploy_speed){
                                           // ---------- Deploy Left-MechaArm (left) >> Deployed State-4 (current draw ~ 0.6A)
// --------------------------------------------------- // Left-MechaArm op-codes // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(8, 60,    deploy_speed, 1);
  move_Servo_Bidirect(10, 0,    deploy_speed, 1);
  move_Servo_Bidirect(9, 0,     deploy_speed, 1);
  move_Servo_Bidirect(11, -90,  deploy_speed, 1);
  move_Servo_Bidirect(10, 90,   deploy_speed, 1);
  move_Servo_Bidirect(8, 118,   deploy_speed, 1);
  move_Servo_Bidirect(9, 110,   deploy_speed, 1);
  move_Servo_Bidirect(11, -110, deploy_speed, 1);
}

void roboDrive_ParkLeft_Arm_fromState4(uint8_t parking_speed){
                                                // ---------- Park Left-MechaArm (left) >> Parked-State 4 (current draw ~ 0.6A)
// ---------------------------------------------------- // Left-MechaArm op-codes // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(13, 10,  parking_speed, 1);
  move_Servo_Bidirect(12, 0,   parking_speed, 1);
  move_Servo_Bidirect(7,  0,   parking_speed, 1);
  move_Servo_Bidirect(10, 0,   parking_speed, 1);
  move_Servo_Bidirect(11, 0,   parking_speed, 1);
  move_Servo_Bidirect(8,  90,  parking_speed, 1);
  move_Servo_Bidirect(9, -20,  parking_speed, 1);
  move_Servo_Bidirect(11, 90,  parking_speed, 1);
  move_Servo_Bidirect(10, 70,  parking_speed, 1);
  move_Servo_Bidirect(9, -57,  parking_speed, 1);
  move_Servo_Bidirect(10, 45,  parking_speed, 1);
  move_Servo_Bidirect(8,  110, parking_speed, 1);
  move_Servo_Bidirect(11, 125, parking_speed, 1);
  move_Servo_Bidirect(10, 54,  parking_speed, 1);
}

void roboDrive_DeployArmsState2(uint8_t deploy_speed){
                                                 // -deploy arms (state_2) (lef/right) >> state 2 (current draw ~ 0.50-0.65A)
// --------------------------------------------- // right mecha arm op-codes,   // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(0, -10,  deploy_speed, 1);
  move_Servo_Bidirect(4,  85,  deploy_speed, 1);
  move_Servo_Bidirect(2,   0,  deploy_speed, 1);
  move_Servo_Bidirect(3,  53,  deploy_speed, 1);
  move_Servo_Bidirect(1, -90,  deploy_speed, 1);

  move_Servo_Bidirect(5, -100, deploy_speed, 1);
  move_Servo_Bidirect(6,  -50, deploy_speed, 1);
// --------------------------------------------------------------------------------------------- //  left mecha arm op-codes
  move_Servo_Bidirect(7,   0,  deploy_speed, 1);
  move_Servo_Bidirect(11, 90,  deploy_speed, 1);
  move_Servo_Bidirect(9,   0,  deploy_speed, 1);
  move_Servo_Bidirect(10, 60,  deploy_speed, 1);
  move_Servo_Bidirect(8,  74,  deploy_speed, 1);

  move_Servo_Bidirect(12,  60, deploy_speed, 1);
  move_Servo_Bidirect(13, -80, deploy_speed, 1);
}

void roboDrive_DeployArmsState3(uint8_t deploy_speed){
                                                 // deploy arms (state_3) (lef/right) >> state 3 (current draw ~ 0.50-0.65A)
// ----------------------------------------- --- // right mecha arm op-codes,  // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(0, -10,  deploy_speed, 1);
  move_Servo_Bidirect(4,  85,  deploy_speed, 1);
  move_Servo_Bidirect(2,   0,  deploy_speed, 1);
  move_Servo_Bidirect(3,  53,  deploy_speed, 1);
  move_Servo_Bidirect(1, -90,  deploy_speed, 1);

  move_Servo_Bidirect(5,  0,   deploy_speed, 1);
  move_Servo_Bidirect(6, 40,   deploy_speed, 1);
// -------------------------------------------------------------------------------------------------- //  left mecha arm op-codes
  move_Servo_Bidirect(7,   0,  deploy_speed, 1);
  move_Servo_Bidirect(11, 90,  deploy_speed, 1);
  move_Servo_Bidirect(9,   0,  deploy_speed, 1);
  move_Servo_Bidirect(10, 60,  deploy_speed, 1);
  move_Servo_Bidirect(8,  74,  deploy_speed, 1);

  move_Servo_Bidirect(12,  0,  deploy_speed, 1);
  move_Servo_Bidirect(13, -8,  deploy_speed, 1);
}

void roboDrive_ActivateArms(uint8_t activation_speed){
                                     //- activate both arms from parked state ((lef/right) >> state 4 (current draw ~ 0.68-0.70A)
// ----------------------------------------------- // right mecha arm op-code,      // speed >> ((superfast) 0 - 255 (very slow))
  move_Servo_Bidirect(0, -10, activation_speed, 1);
  move_Servo_Bidirect(4, 105, activation_speed, 1);
  move_Servo_Bidirect(2, -45, activation_speed, 1);
// ------------------------------------------------------------------------------------------------- //  left mecha arm op-codes
  move_Servo_Bidirect(7,    0, activation_speed, 1);
  move_Servo_Bidirect(11, 115, activation_speed, 1);
  move_Servo_Bidirect(9,  -36, activation_speed, 1);
}

void roboDrive_LookUp(uint8_t speed){
                                                                              // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(14, 55, speed, 1);
}

void roboDrive_LookUp_Gradually(uint8_t speed){

  uint8_t step = 4;
  uint8_t servo_number = 14;

  int8_t servo_number_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_number_current_angle < 55){
    move_Servo_Bidirect(servo_number, (servo_number_current_angle + step), speed, 1);
  }
  else{

  }
}

void roboDrive_LookDown_Gradually(uint8_t speed){

  uint8_t step = 4;
  uint8_t servo_number = 14;

  int8_t servo_number_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_number_current_angle > -15){
    move_Servo_Bidirect(servo_number, (servo_number_current_angle - step), speed, 1);
  }
  else{

  }
}

void roboDrive_LookDown(uint8_t speed){
                                                                              // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(14, -15, speed, 1);
}

void roboDrive_LookUpFront(uint8_t speed){
                                                                              // speed >> ((superfast) 1 - 255 (very slow))
  move_Servo_Bidirect(14, 10, speed, 1);
}

void roboDrive_LiftUpArms(uint8_t speed){
                            // ------------- lift-up arms (^_^) show-off ((lef/right) >> state 4 (current draw ~ 0.68-0.70A)
// ------------------------------------------------ // right mecha-arm op-code,   speed >> ((superfast) 0 - 255 (very slow))
  move_Servo_Bidirect(0, -10, speed, 1);
  move_Servo_Bidirect(4, 105, speed, 1);
  move_Servo_Bidirect(2,   0, speed, 1);
  move_Servo_Bidirect(1, -90, speed, 1);
  move_Servo_Bidirect(3,  -8, speed, 1);
  move_Servo_Bidirect(4,   0, speed, 1);
// ----------- waive mecha-arm ------------------------------------------------------------------------------------------//
  move_Servo_Bidirect(0,  45, speed, 1);
  move_Servo_Bidirect(0, -10, speed, 1);
  move_Servo_Bidirect(0,  45, speed, 1);
  move_Servo_Bidirect(0, -10, speed, 1);
// ----------- nodd tower ----------------------------------------------------------------------------------------------//
  roboDrive_LookUp(speed);
  roboDrive_LookDown(speed);
  roboDrive_LookUpFront(speed);
// ------------------------------------------------------------------------------------------- //  left mecha arm op-codes
  move_Servo_Bidirect(7,   0,  speed, 1);
  move_Servo_Bidirect(11, 115, speed, 1);
  move_Servo_Bidirect(9,    0, speed, 1);
  move_Servo_Bidirect(8,   76, speed, 1);
  move_Servo_Bidirect(10,   0, speed, 1);
  move_Servo_Bidirect(11,   0, speed, 1);
}

void roboDrive_move_Forward_LeftMechaArm(uint8_t speed){

  uint8_t step_1 = 4;
  uint8_t step_2 = 4;
  uint8_t step_3 = 3;
  uint8_t step_4 = 3;

  uint8_t servo_number_1 = 8;
  uint8_t servo_number_2 = 9;
  uint8_t servo_number_3 = 10;
  uint8_t servo_number_4 = 11;

  int8_t servo_number_1_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_1);
  int8_t servo_number_2_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_2);
  int8_t servo_number_3_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_3);
  int8_t servo_number_4_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_4);

  if(servo_number_1_current_angle > 36){
    move_Servo_Bidirect(servo_number_1, (servo_number_1_current_angle - step_1), speed, 1);
  }
  else{

  }

  if(servo_number_2_current_angle < 40){
    move_Servo_Bidirect(servo_number_2, (servo_number_2_current_angle + step_2), speed, 1);
  }
  else{

  }

  if(servo_number_3_current_angle > 10){
    move_Servo_Bidirect(servo_number_3, (servo_number_3_current_angle - step_3), speed, 1);
  }

  else{

  }

  if(servo_number_4_current_angle > 0){
    move_Servo_Bidirect(servo_number_4, (servo_number_4_current_angle - step_4), speed, 1);
  }

  else{

  }
}

void roboDrive_move_Forward_RightMechaArm(uint8_t speed){

  uint8_t step_1 = 4;
  uint8_t step_2 = 4;
  uint8_t step_3 = 3;
  uint8_t step_4 = 3;

  uint8_t servo_number_1 = 1;
  uint8_t servo_number_2 = 2;
  uint8_t servo_number_3 = 3;
  uint8_t servo_number_4 = 4;

  int8_t servo_number_1_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_1);
  int8_t servo_number_2_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_2);
  int8_t servo_number_3_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_3);
  int8_t servo_number_4_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_4);

  if(servo_number_1_current_angle < -36){
    move_Servo_Bidirect(servo_number_1, (servo_number_1_current_angle + step_1), speed, 1);
  }
  else{

  }

  if(servo_number_2_current_angle < 40){
    move_Servo_Bidirect(servo_number_2, (servo_number_2_current_angle + step_2), speed, 1);
  }
  else{

  }

  if(servo_number_3_current_angle > 10){
    move_Servo_Bidirect(servo_number_3, (servo_number_3_current_angle - step_3), speed, 1);
  }

  else{

  }

  if(servo_number_4_current_angle > 0){
    move_Servo_Bidirect(servo_number_4, (servo_number_4_current_angle - step_4), speed, 1);
  }

  else{

  }
}

void roboDrive_Crane_LeftArm(uint8_t speed){

  uint8_t step_1 = 4;
  uint8_t step_2 = 4;
  uint8_t step_3 = 3;
  uint8_t step_4 = 3;

  uint8_t servo_number_1 = 8;
  uint8_t servo_number_2 = 9;
  uint8_t servo_number_3 = 10;
  uint8_t servo_number_4 = 11;

  int8_t servo_number_1_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_1);
  int8_t servo_number_2_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_2);
  int8_t servo_number_3_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_3);
  int8_t servo_number_4_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_4);

  if(servo_number_1_current_angle < 100){
    move_Servo_Bidirect(servo_number_1, (servo_number_1_current_angle + step_1), speed, 1);
  }
  else{

  }

  if(servo_number_2_current_angle < 40){
    move_Servo_Bidirect(servo_number_2, (servo_number_2_current_angle + step_2), speed, 1);
  }
  else{

  }

  if(servo_number_3_current_angle > 10){
    move_Servo_Bidirect(servo_number_3, (servo_number_3_current_angle - step_3), speed, 1);
  }

  else{

  }

  if(servo_number_4_current_angle > 0){
    move_Servo_Bidirect(servo_number_4, (servo_number_4_current_angle - step_4), speed, 1);
  }

  else{

  }
}

void roboDrive_Crane_RightArm(uint8_t speed){

  uint8_t step_1 = 4;
  uint8_t step_2 = 4;
  uint8_t step_3 = 3;
  uint8_t step_4 = 3;

  uint8_t servo_number_1 = 1;
  uint8_t servo_number_2 = 2;
  uint8_t servo_number_3 = 3;
  uint8_t servo_number_4 = 4;

  int8_t servo_number_1_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_1);
  int8_t servo_number_2_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_2);
  int8_t servo_number_3_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_3);
  int8_t servo_number_4_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number_4);

  if(servo_number_1_current_angle > -100){
    move_Servo_Bidirect(servo_number_1, (servo_number_1_current_angle - step_1), speed, 1);
  }
  else{

  }

  if(servo_number_2_current_angle < 40){
    move_Servo_Bidirect(servo_number_2, (servo_number_2_current_angle + step_2), speed, 1);
  }
  else{

  }

  if(servo_number_3_current_angle > 10){
    move_Servo_Bidirect(servo_number_3, (servo_number_3_current_angle - step_3), speed, 1);
  }

  else{

  }

  if(servo_number_4_current_angle > 0){
    move_Servo_Bidirect(servo_number_4, (servo_number_4_current_angle - step_4), speed, 1);
  }

  else{

  }
}

void roboDrive_move_RightMechaArm_CW(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 0;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle < 125){
    move_Servo_Bidirect(servo_number, (servo_current_angle + step), speed, 1);
  }
  else{
  }
}

void roboDrive_move_RightMechaArm_CCW(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 0;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle > -10){
    move_Servo_Bidirect(servo_number, (servo_current_angle - step), speed, 1);
  }
  else{
  }
}

void roboDrive_move_LeftMechaArm_CW(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 7;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle < 10){
    move_Servo_Bidirect(servo_number, (servo_current_angle + step), speed, 1);
  }
  else{
  }
}

void roboDrive_move_LeftMechaArm_CCW(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 7;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle > -125){
    move_Servo_Bidirect(servo_number, (servo_current_angle - step), speed, 1);
  }
  else{
  }
}

void roboDrive_CloseRightGripper_gradually(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 6;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle < 60){
    move_Servo_Bidirect(servo_number, (servo_current_angle + step), speed, 1);
  }
  else{
  }
}

void roboDrive_OpenRightGripper_gradually(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 6;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle > -65){
    move_Servo_Bidirect(servo_number, (servo_current_angle - step), speed, 1);
  }
  else{
  }
}

void roboDrive_CloseLeftGripper_gradually(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 13;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle < 10){
    move_Servo_Bidirect(servo_number, (servo_current_angle + step), speed, 1);
  }
  else{
  }
}

void roboDrive_OpenLeftGripper_gradually(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 13;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle > -120){
    move_Servo_Bidirect(servo_number, (servo_current_angle - step), speed, 1);
  }
  else{
  }
}

bool roboDrive_OpenGripperRightMechaArm(uint8_t speed){
                                            // positive angles "close" gripper, negative angles "open" the gripper
                                                                     // speed >> ((superfast) 1 - 255 (very slow))
  if(gripper_State != 1){
    move_Servo_Bidirect(6, 40, speed, 1);
    gripper_State = 1;
    return true;
  }
  else{
    move_Servo_Bidirect(6, -55, speed, 1);
    gripper_State = 0;
    return false;
  }
}

bool roboDrive_OpenGripperLeftMechaArm(uint8_t speed){
                                            // positive angles "close" gripper, negative angles "open" the gripper
                                                                     // speed >> ((superfast) 1 - 255 (very slow))
  if(gripper_State != 1){
    move_Servo_Bidirect(13, -90, speed, 1);
    gripper_State = 1;
    return true;
  }
  else{
    move_Servo_Bidirect(13, 0, speed, 1);
    gripper_State = 0;
    return false;
  }
}

void roboDrive_Rotate_CW_GripperRightMechaArm(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 5;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle < 125){
    move_Servo_Bidirect(servo_number, (servo_current_angle + step), speed, 1);
  }
  else{
  }
}

void roboDrive_Rotate_CCW_GripperRightMechaArm(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 5;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle > -125){
    move_Servo_Bidirect(servo_number, (servo_current_angle - step), speed, 1);
  }
  else{
  }
}

void roboDrive_Rotate_CW_GripperLeftMechaArm(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 12;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle < 125){
    move_Servo_Bidirect(servo_number, (servo_current_angle + step), speed, 1);
  }
  else{
  }
}

void roboDrive_Rotate_CCW_GripperLeftMechaArm(uint8_t speed){

  uint8_t step = 5;
  uint8_t servo_number = 12;
  int8_t servo_current_angle = FRAM_readByte(SERVOS_POSITION_ADDRESS + servo_number);
  if(servo_current_angle > -125){
    move_Servo_Bidirect(servo_number, (servo_current_angle - step), speed, 1);
  }
  else{
  }
}

void roboDrive_Deploy_FlatSurfaceProbe(uint8_t deploy_speed){
  // -deploy left-arm with test-probe (left) >> state probe 90° (current draw ~     )
// --------------------------------------------- // right mecha arm op-codes,   // speed >> ((superfast) 1 - 255 (very slow))
// move_Servo_Bidirect(0, -10,  deploy_speed, 1);
// move_Servo_Bidirect(4,  85,  deploy_speed, 1);
// move_Servo_Bidirect(2,   0,  deploy_speed, 1);
// move_Servo_Bidirect(3,  53,  deploy_speed, 1);
// move_Servo_Bidirect(1, -90,  deploy_speed, 1);
//
// move_Servo_Bidirect(5, -100, deploy_speed, 1);
// move_Servo_Bidirect(6,  -50, deploy_speed, 1);
// --------------------------------------------------------------------------------------------- //  left mecha arm op-codes
move_Servo_Bidirect(7,   0,  deploy_speed, 1);
move_Servo_Bidirect(11, 90,  deploy_speed, 1);
move_Servo_Bidirect(9,   0,  deploy_speed, 1);
move_Servo_Bidirect(10,  0,  deploy_speed, 1);
move_Servo_Bidirect(8,  100,  deploy_speed, 1);
move_Servo_Bidirect(9,  120,  deploy_speed, 1);

move_Servo_Bidirect(12,  60, deploy_speed, 1);
move_Servo_Bidirect(13, -80, deploy_speed, 1);
}
// ----------------------------------- Arm Control ---------------------------------------------------------//
// ---------------------------------------------------------------------------------------------------------//
// ----------------------------------- Automatic Mode ------------------------------------------------------//
void automatic_Drive(void){                // obstacle avoidance sub-routine, exploits IRED-sensor to navigate

    uint16_t pingData[8];
    uint16_t x_0, x_45, x_90, x_135, x_180;
    uint16_t x_minus_45, x_minus_90, x_minus_135;
// ----------------------------------- clockwise -----------------------------------------------------------//
    x_minus_135 = fetchDataADC(IRED_SENSOR);                              // turn left - 335° sensor at - 135°
    stepperDrive_adapted(50, 100);
    _delay_ms(200);
    x_0 = fetchDataADC(IRED_SENSOR);                                          // front view 0°    sensor at 0°
    _delay_ms(50);
    stepperDrive_adapted(50, 100);
    _delay_ms(200);
    x_45 = fetchDataADC(IRED_SENSOR);                                        // turn right +45°  sensor at 45°
    _delay_ms(50);
    stepperDrive_adapted(50, 100);
    _delay_ms(200);
    x_90 = fetchDataADC(IRED_SENSOR);                                       // turn right +45°   sensor at 90°
    _delay_ms(50);
    stepperDrive_adapted(50, 100);
    _delay_ms(200);
    x_135 = fetchDataADC(IRED_SENSOR);                                     // turn right +45°   sensor at 135°
    _delay_ms(50);
// ------------------------------- right side scan saved -------------------------------------------------- //
// -------------------------------------------------------------------------------------------------------- //
// -------------------------------- counter clockwise ----------------------------------------------------- //
    stepperDrive_adapted(-120, 90);
    stepperDrive_adapted(-120, 90);
    _delay_ms(200);
    x_180 = fetchDataADC(IRED_SENSOR);                                     // turn right +45°   sensor at 180°
    _delay_ms(50);
    stepperDrive_adapted(-64, 90);
    _delay_ms(200);
    x_minus_45 = fetchDataADC(IRED_SENSOR);                               // turn left - 225° sensor at - 45°
    _delay_ms(50);
    stepperDrive_adapted(-60, 90);
    _delay_ms(200);
    x_minus_90 = fetchDataADC(IRED_SENSOR);                               // turn left - 270° sensor at - 90°
    _delay_ms(200);
    stepperDrive_adapted(120, 90);
    _delay_ms(50);
    stepperDrive_adapted(44, 90);
// ---------------------------------- left side scan saved ------------------------------------------------//
      pingData[0] = x_0;
      pingData[1] = x_45;
      pingData[2] = x_90;
      pingData[3] = x_135;
      pingData[4] = x_180;
      pingData[5] = x_minus_45;
      pingData[6] = x_minus_90;
      pingData[7] = x_minus_135;

      scanner_minVal = find_Min(pingData, 8);

      printString("scanner minimal value: ");
      printWord(scanner_minVal);
      printString("\r\n");

      for(int i = 0; i < 8; i ++){
        printWord(pingData[i]);
        printString("\t");
      }
//    if(pingData[0] <= 100){
//            printString("0 deg view: >> clean <<\t");
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//    }
//    if(pingData[1] <= 100){
//            printString("45 deg view: >> clean <<\t");
//            roboDrive("right", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//            _delay_ms(2000);
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//      }
//    if(pingData[2] <= 100){
//            printString("90 deg view: >> clean <<\t");
//            roboDrive("right", 36);
//            _delay_ms(3600);
//            roboDrive("brake", 0);
//            _delay_ms(5000);
//            roboDrive("forward", 36);
//            _delay_ms(5000);
//            roboDrive("brake", 0);
//      }
//    if(pingData[3] <= 100){
//            printString("135 deg view: >> clean <<\t");
//            roboDrive("right", 36);
//            _delay_ms(5000);
//            roboDrive("brake", 0);
//            _delay_ms(2000);
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//      }
//    if(pingData[4] <= 100){
//            printString("180 deg view: >> clean <<\t");
//            roboDrive("right", 36);
//            _delay_ms(5000);
//            _delay_ms(5000);
//            roboDrive("brake", 0);
//            _delay_ms(2000);
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//      }
//    if(pingData[5] <= 100){
//            printString("-45 deg view: >> clean <<\t");
//            roboDrive("left", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//            _delay_ms(2000);
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//      }
//    if(pingData[6] <= 100){
//            printString("-90 deg view: >> clean <<\t");
//            roboDrive("left", 36);
//            _delay_ms(3600);
//            roboDrive("brake", 0);
//            _delay_ms(2000);
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//      }
//    if(pingData[7] <= 100){
//            printString("-135 deg view: >> clean <<\t");
//            roboDrive("right", 36);
//            _delay_ms(5000);
//            roboDrive("brake", 0);
//            _delay_ms(2000);
//            roboDrive("forward", 36);
//            _delay_ms(2000);
//            roboDrive("brake", 0);
//      }
//      else{
//            printString(">> Exception Raised!!!! <<\t");
//            //printString("\n>> trying again .. <<\t");
//            //automatic_Drive();
//      }
}
// --------------------------------------- Informing Functions -----------------------------------------------//
void printTime(uint16_t year_, uint8_t months_, uint8_t days_, uint8_t hours_, uint8_t minutes_, uint8_t seconds_){
                                                      // printTime() function prints the actual time in terminal
    printString("Year: ");
    printWord(year_);                                                                             // prints year
    transmitByte(':');                                                                        // prints ':' dots
    transmitByte(' ');
    printString("Month: ");
    printByte(months_);                                                                          // prints month
    transmitByte(':');                                                                        // prints ':' dots
    transmitByte(' ');
    printString("Day: ");
    printByte(days_);                                                                             // prints days
    transmitByte(':');                                                                        // prints ':' dots
    transmitByte(' ');
    printString("Hour: ");
    printByte(hours_);                                                                           // prints hours
    transmitByte(':');                                                                        // prints ':' dots
    transmitByte(' ');
    printString("Min: ");
    printByte(minutes_);                                                                       // prints minutes
    transmitByte(':');                                                                        // prints ':' dots
    transmitByte(' ');
    printString("Sec: ");
    printByte(seconds_);                                                                       // prints seconds
    transmitByte('\r');
    transmitByte('\n');
}

static inline void printTemperature(uint8_t tempReading) {
                                                                            // temperature stored as 2 x Celsius
  printByte((tempReading >> 1));
  sprintf(temp_buffer,"%i", (tempReading >> 1));
  //lcd_print_str("     ");
  //lcd_write("t/C = %i", (tempReading >> 1));
  lcd_print_str(temp_buffer);
  if (tempReading & 1) {
    printString(".5\r\n");
    lcd_print_str(".5");
    lcd_print_str("      ");
  }
  else {
    printString(".0\r\n");
    lcd_print_str(".0");
    lcd_print_str("      ");
  }
}

void printFloat(float number){

    number = round(number * 100) / 100;                                        // rounds off to 2 decimal places

    transmitByte('0' + number / 10);                                                               // tens place
    transmitByte('0' + number - 10 * floor(number / 10));                                                // ones
    transmitByte('.');
    transmitByte('0' + (number * 10) - floor(number) * 10);                                            // tenths
    transmitByte('0' + (number * 100) - floor(number * 10) * 10);                             //hundredths place
    printString("\r\n");
}

void distanceTraveled(short right_pulse, short left_pulse){

    printString("distance-traveled: ");      // sub-program, interrupt driven to keep track of traveled distance
    printString("right = ");                                     // reads data from sensors and presents to user
    printWord(right_pulse);
    transmitByte(' ');                                                                  // for right Hall-sensor
    transmitByte(':');
    transmitByte(' ');
    printString("left = ");
    printWord(left_pulse);                                                               // for left Hall-sensor
    printString("\t");
}

void playNote(uint16_t period, uint16_t duration){                             // generates various pitch sounds

    uint16_t elapsed;
    uint16_t i;
    for ( elapsed = 0; elapsed < duration; elapsed += period ){    // loop with variable delay selects the pitch
            for (i = 0; i < period; i ++) {
                _delay_us(1);
    }
    STEPPER_PORT ^= (1 << SPEAKER);                             // STEPPER_PORT and speaker share same registers
  }
}

void help_Readme(){

  playNote(notes[0], currentNoteLength);
  playNote(notes[2], currentNoteLength);
  playNote(notes[10], currentNoteLength);

  lcd_clear();
  lcd_home();
  lcd_print_str("roboDrive 5.8 ");
  lcd_print_str("Help / Readme");

  printString("\r\n\n ---> [roboDrive Engine Instruction set] <---\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n   -------  > Run Control < ----------------------\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n -> press [T] (Update Time/Date)\r\n");
  printString("\r\n -> press [O] (Configure Kinetics and Parameters)\r\n");
  printString("\r\n -> press [B] (Start Vibrations Scanning)\r\n");
  printString("\r\n -> press [E] (Start >[IR]< Distance Measurement)\r\n");
  printString("\r\n -> press [Q] (Start >[LASER]< Distance Scan on LIDAR)\r\n");
  printString("\r\n -> press [Z] (Start a Temperature Scan)\r\n");
  printString("\r\n -> press [U] (View Telemetry)\r\n");
  printString("\r\n -> press [F] (Memory Read/Write Operations)\r\n");
  printString("\r\n -> press [H] (Help/Readme)\r\n\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n   -------  > Drivetrain Control < ----------------------\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n -> press [W] (Forward Drive)\r\n");
  printString("\r\n -> press [S] (Reverse Drive/Go-Back)\r\n");
  printString("\r\n -> press [A] (Turn Left)\r\n");
  printString("\r\n -> press [D] (Turn Right)\r\n");
  printString("\r\n -> press [X] (Stop|Brake)\r\n\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n   -------  > Tower Control < ----------------------------\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n -> press [4] (Look Left)\r\n");
  printString("\r\n -> press [6] (Look Right)\r\n");
  printString("\r\n -> press [8] (Look Up)\r\n");
  printString("\r\n -> press [5] (Look Up-Front)\r\n");
  printString("\r\n -> press [0] (Look Down)\r\n");
  printString("\r\n -> press [SHIFT]+[*] (Light OFF)\r\n");
  printString("\r\n -> press [SHIFT]+[/] (Light ON)\r\n\n");
  printString("\r\n -> cycle [SHIFT]+[/] with [SHIFT]+[*] (Mode Change)\r\n\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n   -------  > Manipulator Control < ----------------------\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n -> press [#] (Recover Both Arms from random position)\r\n");
  printString("\r\n -> press [7] (Recover Left-Arm  from random position (arm has been touched/moved by accident in OFF state))\r\n");
  printString("\r\n -> press [9] (Recover Right-Arm from random position (arm has been touched/moved by accident in OFF state))\r\n");
  printString("\r\n -> press [Y] (Custom Servo Commands)\r\n");
  printString("\r\n -> press [k] (Park Both Arms)\r\n");
  printString("\r\n -> press [L] (Park Left  Mecha-Arm)\r\n");
  printString("\r\n -> press [R] (Park Right Mecha-Arm)\r\n");
  printString("\r\n -> press [a] (Deploy State-1 Left  Mecha-Arm)\r\n");
  printString("\r\n -> press [l] (Deploy State-1 Right Mecha-Arm)\r\n");
  printString("\r\n -> press [SHIFT]+[-] (Activate Both Arms)\r\n");
  printString("\r\n -> press [SHIFT]+[+] (Lift-Up Both Arms)\r\n\n");
  printString("\r\n -> press [1] (Deploy Both Arms > State-1)\r\n");
  printString("\r\n -> press [2] (Deploy Both Arms > State-2)\r\n");
  printString("\r\n -> press [3] (Deploy Both Arms > State-3)\r\n");
  printString("\r\n -> press [c] (Steer Left Mecha-Arm CW)\r\n");
  printString("\r\n -> press [v] (Steer Left Mecha-Arm CCW)\r\n");
  printString("\r\n -> press [n] (Steer Right Mecha-Arm CW)\r\n");
  printString("\r\n -> press [m] (Steer Right Mecha-Arm CCW)\r\n");
  printString("\r\n -> press [e] (Extend Forward >> Gradually Left  Mecha-Arm )\r\n");
  printString("\r\n -> press [r] (Crane Backward >> Gradually Left  Mecha-Arm )\r\n");
  printString("\r\n -> press [o] (Extend Forward >> Gradually Right Mecha-Arm )\r\n");
  printString("\r\n -> press [p] (Crane Backward >> Gradually Right Mecha-Arm )\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n   -------  > Gripper Control < --------------------------\r\n");
  printString("\r\n   --------------------------------------------\r\n");
  printString("\r\n -> press [d] (Open  Gradually >> Left  Gripper)\r\n");
  printString("\r\n -> press [f] (Close Gradually >> Left  Gripper)\r\n");
  printString("\r\n -> press [h] (Open  Gradually >> Right Gripper)\r\n");
  printString("\r\n -> press [j] (Close Gradually >> Right Gripper)\r\n");
  printString("\r\n -> press [C] (Rotate Left  Mecha-Arm Gripper CW )\r\n");
  printString("\r\n -> press [V] (Rotate Left  Mecha-Arm Gripper CCW)\r\n");
  printString("\r\n -> press [N] (Rotate Right Mecha-Arm Gripper CW )\r\n");
  printString("\r\n -> press [M] (Rotate Right Mecha-Arm Gripper CCW)\r\n");
  printString("\r\n   -------------------------------------------------------\r\n");
}

//----------------------------------------- Timing Functions -------------------------------------------------//
void monthsRun(void){                                                 // small function counts months from Timer

    months_++;
    if(months_ > 12){
        months_ = 0;
        year_ += 1;
    }
}

void daysRun(void){                                                     // small function counts days from Timer

    days_ ++;
    if(days_ > 31){
        days_ = 0;
        monthsRun();
    }
}

void hoursRun(void){                                                   // small function counts hours from Timer

    hours_ ++;
    if(hours_ > 23){
        hours_ = 0;
        daysRun();
    }

    playNote(notes[14], currentNoteLength);
    _delay_ms(100);
    playNote(notes[5], currentNoteLength);
    _delay_ms(100);
    playNote(notes[5], currentNoteLength);
    _delay_ms(100);
    playNote(notes[14], currentNoteLength);
    _delay_ms(100);
    playNote(notes[14], currentNoteLength);
}

void minutesRun(void){                                                // small function counts minutes from Timer

    minutes_ ++;
    if(minutes_ > 59){
        minutes_ = 0;
        hoursRun();
    }
    lcd_clear();
    playNote(notes[10], currentNoteLength);
    _delay_ms(100);
    playNote(notes[1], currentNoteLength);
    _delay_ms(100);
    playNote(notes[1], currentNoteLength);

    power_pack_voltage = oversample16x() * VOLTAGE_DIV_FACTOR * REF_VCC / 4096;          // power supply voltage
    printString("power-pack voltage/(Volts) = ");
    printFloat(power_pack_voltage);
    sprintf(voltage_buffer, "%0.2f", power_pack_voltage);
    lcd_clear();
    lcd_home();
    lcd_print_str("Charge: ");
    lcd_print_str(voltage_buffer);
    lcd_print_str("V  ");


    LIDAR_scanArea();
    LIDAR_status = check_LIDAR_busyFlag();
    if((LIDAR_status & LIDAR_LITE_BITMASK) == 0){
            printString("distance = ");
            LIDAR_distance = LIDAR_shuttle_Data();
            printWord(LIDAR_distance);
            printString("\r\n");
    }
    else{
            printString("LIDAR is Busy....\r\n");
            LIDAR_status = check_LIDAR_busyFlag();
    }

    sprintf(LIDAR_range_buffer, "%d", LIDAR_distance);
    //lcd_home();
    lcd_print_str("Range: ");
    lcd_print_str(LIDAR_range_buffer);
    lcd_print_str("cm");

    if(power_pack_voltage >= 15){

      //pcf8575_Output((1 << LED_0) | (1 << LED_1) | (1 << LED_2) | (1 << LED_3));       // battery full, green LEDs
      pcf8575_Output(0b1111111111110000);                              // LEDs No 10 > 7 // battery full, green LEDs
    }
    else if(power_pack_voltage >= 14){

      //pcf8575_Output((1 << LED_1) | (1 << LED_2) | (1 << LED_3));       // battery partially full, some green LEDs
      //pcf8575_Output((~ LED_1) | ( ~ LED_2) | (~ LED_3));               // battery partially full, some green LEDs
      pcf8575_Output(0b1111111111100000);
    }
    else if(power_pack_voltage >= 13){

      //pcf8575_Output((1 << LED_2) | (1 << LED_3));                      // battery more than half, some green LEDs
      //pcf8575_Output(( ~ LED_2) | ( ~ LED_3));                          // battery more than half, some green LEDs
      pcf8575_Output(0b1111111111000011);
    }
    else if(power_pack_voltage >= 10){

      //pcf8575_Output((1 << LED_4) |(1 << LED_5));                               // battery about half, yellow LEDs
      //pcf8575_Output(( ~ LED_4) | ( ~ LED_5));                                  // battery about half, yellow LEDs
      pcf8575_Output(0b011111111000111);
    }
    else if(power_pack_voltage < 10){

      //pcf8575_Output((1 << LED_5));                                         // battery less than half, yellow LEDs
      //pcf8575_Output(( ~ LED_5));                                           // battery less than half, yellow LEDs
      pcf8575_Output(0b1011111110011111);
    }
    else{

      //pcf8575_Output((1 << LED_6) |(1 << LED_7));                                    // battery low ...., red LEDs
      //pcf8575_Output(( ~ LED_6) |( ~ LED_7));                                        // battery low ...., red LEDs
      pcf8575_Output(0b0011111101111111);
    }
}

void secondsRun(void){                                                   // small function counts seconds from Timer

    seconds_ ++;
    if(seconds_ > 59){
        seconds_ = 0;
        minutesRun();
    }
    playNote(notes[3], currentNoteLength);
    _delay_ms(50);
    playNote(notes[5], currentNoteLength);
}
// ----------------------------------- under construction ----------------------------------------------------- //
//void millisecondsRun(void){
//
//    milliseconds_ ++;
//    if(milliseconds_ > 999){
//        milliseconds_ = 0;
//        secondsRun();
//    }
//    //printWord(milliseconds_);
//    //printString("\r\n");
//}
//----------------------------------- Monitoring Functions -----------------------------------------------------//
void telemetryMotorRight(void){                          // sub-program to connect to motor sensors and fetch data

    move_knob_R = (HALL_PIN & (1 << HALL_S1R));
    move_dir_R = (HALL_PIN & (1 << HALL_S2R));

    if(move_knob_R == 0){

      if(move_dir_R == 1){

        hall_S2R_pulse ++;
      }
    else{

      hall_S2R_pulse --;
    }
  }
}

void telemetryMotorLeft(void){                          // sub-program to connect to motor sensors and fetch data

    move_knob_L = (HALL_PIN & (1 << HALL_S1L));
    move_dir_L = (HALL_PIN & (1 << HALL_S2L));

    if(move_knob_L == 0){
      if(move_dir_L == 0){
        hall_S2L_pulse ++;
          }
    else{
        hall_S2L_pulse --;
    }
  }
}

void set_Time(void){

  lcd_clear();
  lcd_home();
  lcd_print_str("> Date/Time");
  playNote(notes[15], currentNoteLength);
  _delay_ms(50);
  playNote(notes[15], currentNoteLength);
  _delay_ms(50);
  playNote(notes[15], currentNoteLength);
  _delay_ms(1000);
  printString("\r\n>> connected to [Kenji-X1]...\r\n");
  printString("\r\n-------------------------------\r\n");
  printString(">> setting date/time...\r\n");
  playNote(notes[15], currentNoteLength);
  printString("\r\nenter month: ");
  lcd_clear();
  lcd_home();
  lcd_print_str("> month");
  months_ = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\r\nenter day: ");
  lcd_clear();
  lcd_home();
  lcd_print_str("> day");
  days_ = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\r\nenter hour: ");
  lcd_clear();
  lcd_home();
  lcd_print_str("> hour");
  hours_ = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\r\nenter minutes: ");
  lcd_clear();
  lcd_home();
  lcd_print_str("> minutes");
  minutes_ = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\r\nenter seconds: ");
  lcd_clear();
  lcd_home();
  lcd_print_str("> seconds");
  playNote(notes[15], currentNoteLength);
  seconds_ = getNumber();
  playNote(notes[15], currentNoteLength);
  sys_tick = 0;
  printString("\r\n");
  printString("\r\n>> roboDrive: time/date successfully updated! \r\n");
  printString(">> roboDrive: thank you\r\n");
  lcd_clear();
  lcd_home();
  lcd_print_str("> done!");
  _delay_ms(1500);
  lcd_clear();
}

void set_Kinetics(void){

  lcd_clear();
  lcd_home();
  lcd_print_str("> roboDrive <   ");
  lcd_print_str("  setup");
  playNote(notes[0], currentNoteLength);
  playNote(notes[2], currentNoteLength);
  playNote(notes[10], currentNoteLength);
  printString("\r\n>> connected to [Kenji-X1]...\r\n");
  printString("\r\n-------------------------------\r\n");
  printString(">> robo-Drive: override current kinetic parameters ...\r\n\n");
  printString(">> enter vehicle ground speed: ");
  _delay_ms(1000);
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_ground");
  drive_speed = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\n>> enter tower rotation angular velocity: ");
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_angular");
  stepper_speed = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\n>> enter tower vertical speed: ");
  _delay_ms(1000);
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_vertical");
  servo_speed_towerdrive = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\n>> enter mecha-arms movement speed: ");
  _delay_ms(1000);
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_mecha-arms");
  servo_speed_mecha_arms = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\n>> enter mecha-arms turning speed (L/R): ");
  _delay_ms(1000);
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_mecha-arms  ");
  lcd_print_str("> V_angular");
  servo_speed_turning_mecha_arms = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\n>> enter grippers clamp speed: ");
  _delay_ms(1000);
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_grippers");
  servo_speed_grippers = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\n>> enter grippers rotation angular velocity: ");
  _delay_ms(1000);
  lcd_clear();
  lcd_home();
  lcd_print_str("> V_angular _ _ ");
  lcd_print_str("> grippers (L/R)");
  servo_speed_rotation_grippers = getNumber();
  playNote(notes[15], currentNoteLength);
  printString("\r\n\n>> robo-Drive: kinetic parameters successfully updated! \r\n");
  printString(">> robo-Drive: thank you\r\n");
  lcd_clear();
  lcd_home();
  lcd_print_str("kinetics updated");
  lcd_print_str("> done!");
  _delay_ms(1500);
  lcd_clear();
}

//----------------------------------- Scanner and Data Acquisition Functions ------------------------------------//
void vibration_Scanning(void){

  lcd_clear();
  lcd_home();
  lcd_print_str("> Vibration     ");
  lcd_print_str("  Scanner  ");
  playNote(notes[0],  currentNoteLength);
  playNote(notes[2],  currentNoteLength);
  playNote(notes[10], currentNoteLength);
  printString("\r\n>> connected to [Kenji-X1]...\r\n");
  printString("\r\n-------------------------------\r\n");
  printString(">> roboDrive: [vibrations scanner]\r\n");
  playNote(notes[15], currentNoteLength);
  printString("roboDrive: system ready ... \r\n");
  playNote(notes[15], currentNoteLength);
  printString("start a measurement? (y/n)\r\n");
  user_input = receiveByte();
  playNote(notes[15], currentNoteLength);
  if(user_input == 'y'){
    printString(">> roboDrive: pre-start warm-up... \r\n");
    printString(">> roboDrive: enter number of cycles: (1-99999)\r\n");
    playNote(notes[15], currentNoteLength);
    test_cycle_num = get_32BitNumber();
    printString("\nyou entered : ");
    printWord(test_cycle_num);
    printString("\r\n");
    lcd_clear();
    lcd_home();
    lcd_print_str(">OK!. scanning...");
    theta_0 = 0;
    theta_1 = 0;
    for(i = 0; i <= test_cycle_num; i ++){
      measure_Vibrations();
      theta_1 = theta_1 + theta_0;
      printString("time [ms] = ");
      printWord_32(theta_1);
      printString("\r\n");
    }
    printString("\r\n>> roboDrive: scan complete!\r\n");
    lcd_clear();
    lcd_home();
    lcd_print_str("> scan complete");
  }
  else{
    playNote(notes[15], currentNoteLength);
    printString("measurement canceled by [user]!...\r\n");
  }
}
//void read_EEPROM(void){
//
//    playNote(notes[0], currentNoteLength);
//    playNote(notes[2], currentNoteLength);
//    playNote(notes[10], currentNoteLength);
//    lcd_clear();
//    lcd_home();
//    lcd_print_str("> E2PROM Memory");
//    printString("\r\n>> roboDrive: [E2PROM Memory]\r\n");
//    printString("\r\n-----------------------------\r\n");
//    printString(">> roboDrive: dumping memory...\r\n");
//    printString("\r\n|>    address  <|>    value    <|\r\n");
//
//    for (i_ = 0; i_ <= 50; i_ ++) {                                       // print out first ten bytes of memory
//      printString("|\t");
//      printByte(i_);
//      printString("\t|\t");
//      printByte(EEPROM_readByte(i_));
//      printString("\t|\t");
//      printString("\r\n");
//    }
//    printString(">> roboDrive: [e] to erase all memory\r\n");
//    printString(">> roboDrive: [w] to write byte to memory\r\n\r\n");
//
//    switch (receiveByte()) {                                                                      // take input
//    case 'e':
//      printString(">> roboDrive: clearing E2PROM, this could take a few seconds.\r\n");
//      EEPROM_clearAll();
//      printString("\r\n>> roboDrive: [done!...]\r\n");
//      break;
//    case 'w':
//      printString(">> roboDrive: which memory slot would you like to write to?\r\n");
//      address = getNumber();
//      printString("\r\n>> roboDrive: what number would you like to store there?\r\n");
//      i_ = getNumber();
//      EEPROM_writeByte(address, i_);
//      printString("\r\n");
//      break;
//    default:
//      printString(">> roboDrive: what??\r\n");
//      }
//}

void read_FRAM(void){

    playNote(notes[0], currentNoteLength);
    playNote(notes[2], currentNoteLength);
    playNote(notes[10], currentNoteLength);
    lcd_clear();
    lcd_home();
    lcd_print_str(">> FRAM Memory");
    printString("\r\n>> roboDrive: [FRAM Memory]\r\n");
    printString("\r\n-----------------------------\r\n");
    printString(">> roboDrive: dumping memory...\r\n");
    printString("\r\n|>    address  <|>    value    <|\r\n");

    for (i_ = 0; i_ <= 50; i_ ++) {                                          // print out first ten bytes of memory
      printString("|\t");
      printByte(i_);
      printString("\t|\t");
      printByte(FRAM_readByte(i_));
      printString("\t|\t");
      printString("\r\n");
    }
    printString(">> roboDrive: [e] to erase all memory\r\n");
    printString(">> roboDrive: [w] to write byte to memory\r\n\r\n");
//----------------------------------- Scanner and Data Acquisition Functions ------------------------------------//
    switch (receiveByte()) {                                                                          // take input
    case 'e':
      printString(">> roboDrive: erasing FRAM contents, this could take a few seconds.\r\n");
      FRAM_clearAll();
      printString("\r\n>> roboDrive: [done!...]\r\n");
      break;
    case 'w':
      printString(">> roboDrive: which memory slot would you like to write to?\r\n");
      address = getNumber();
      printString("\r\n>> roboDrive: what number would you like to store there?\r\n");
      i_ = getNumber();
      FRAM_writeByte(address, i_);
      printString("\r\n");
      break;
    default:
      printString(">> roboDrive: what??\r\n");
      }
}

void temperatureScanner(void){

    uint16_t cycle_count = 0;
    uint16_t counter = 0;
    secondsDelay = FRAM_readWord(SECONDS_POINTER);                      // Delay to allow input to enter main menu
    lcd_home();
    lcd_clear();
    lcd_print_str("hit [m] within  ");
    lcd_write("%i sec. for menu", MENU_DELAY);
    printString("*** Press [m] within ");
    printByte(MENU_DELAY);
    printString(" seconds to enter menu. ***\r\n ");
    for (i = 0; i < MENU_DELAY; i++) {
            _delay_ms(1000);
  }
  if (bit_is_set(UCSR0A, RXC0) && (UDR0 == 'm')) {
        enterMenu = 1;
  }
  else {
        enterMenu = 0;
  }
  while (enterMenu) {
        printString("\r\n  ========[  Logging Thermometer (°C) ]=======\r\n");
        lcd_clear();
        lcd_home();
        lcd_print_str("=[Thermometer(C/K)]=");
        currentMemoryLocation = FRAM_readWord(CURRENT_LOCATION_POINTER);
        printString("  ");
        printWord(currentMemoryLocation - MEMORY_START);
        printString(" - readings in log.\r\n");
        printString("  ");
        printWord(secondsDelay);
        printString(" - seconds between readings.\r\n");
        printString(" [<] to shorten sample delay time\r\n");
        printString(" [>] to increase sample delay time\r\n");
        printString(" [?] to reset delay time to 60 sec\r\n");
        printString(" [d] to print out log over serial\r\n");
        printString(" [e] to erase memory\r\n");
        printString(" [s] to start logging\r\n\r\n");

  switch (receiveByte()) {
      case 'd':
      FRAM_SELECT;
      SPI_tradeByte(FRAM_READ);
      FRAM_send24BitAddress(MEMORY_START);
      for (i = MEMORY_START; i < currentMemoryLocation; i++) {
        SPI_tradeByte(0);
        printTemperature(SPDR);
        }
        FRAM_DESELECT;
        break;
      case '<':
          if (secondsDelay >= 10) {
                secondsDelay -= 5;
                FRAM_writeWord(SECONDS_POINTER, secondsDelay);
          }
          break;
      case '>':
          if (secondsDelay < 65000) {
                secondsDelay += 5;
                FRAM_writeWord(SECONDS_POINTER, secondsDelay);
          }
          break;
      case '?':
          secondsDelay = 60;
          FRAM_writeWord(SECONDS_POINTER, secondsDelay);
          break;
          case 'e':
              printString("Clearing FRAM, this could take a few seconds.\r\n");
              FRAM_clearAll();
              FRAM_writeWord(CURRENT_LOCATION_POINTER, MEMORY_START);
              FRAM_writeWord(SECONDS_POINTER, secondsDelay);
              break;
          case 's':
              printString(" >> enter number of cycles: ");
              cycle_count = get_16BitNumber();
              printString("\r\n");
              printString(" >> OK!, logging....\r\n");
              lcd_clear();
              lcd_home();
              lcd_print_str("OK!, logging..");
              _delay_ms(2000);
              enterMenu = 0;
              break;
          default:
              printString("Sorry, didn't understand that.\r\n");
              }
            }
            lcd_clear();
            lcd_home();

            while(counter <= cycle_count) {
                currentMemoryLocation = FRAM_readWord(CURRENT_LOCATION_POINTER);

                i2cStart();                                                 // Make sure LM-75 in temperature mode
                i2cSend(LM75_ADDRESS_W);
                i2cSend(LM75_TEMP_REGISTER);
                                                                                      // Get Temp from thermometer
                i2cStart();                                               // Setup and send address, with read bit
                i2cSend(LM75_ADDRESS_R);
                tempHighByte = i2cReadAck();                                           // two bytes of temperature
                tempLowByte = i2cReadNoAck();
                i2cStop();
                i2cStart();
                i2cSend(LM75_ADDRESS_W);
                i2cSend(LM75_TEMP_REGISTER);
                                                                                      // Get Temp from thermometer
                i2cStart();                                               // Setup and send address, with read bit
                i2cSend(LM75_ADDRESS_R);
                tempHighByte = i2cReadAck();                                           // two bytes of temperature
                tempLowByte = i2cReadNoAck();
                i2cStop();
                                                     //temperatureByte now contains 2 x the temperature in Celsius
                temperatureByte = (tempHighByte << 1) | (tempLowByte >> 7);
                printTemperature(temperatureByte);                                                // serial output
//                lcd_home();
//                lcd_write("temp = %i", (temperatureByte >> 1));
                FRAM_writeByte(currentMemoryLocation, temperatureByte);          // Save the new temperature value

                if (currentMemoryLocation < FRAM_BYTES_MAX) {
                        currentMemoryLocation++;
                           // move on to next location and record new position if not already at the end of memory
                        FRAM_writeWord(CURRENT_LOCATION_POINTER, currentMemoryLocation);

                        for (i = 0; i < secondsDelay; i++) {
                                _delay_ms(1000);                                                           //delay
                                //LED_PORT ^= (1 << LED0);                                // blink to show working
                                }
                        }
                        counter ++;                                                 // increment counter each time
                    }
                    printString("Scan Finished!... (^_^)\r\n");
                    lcd_clear();
                    lcd_home();
                    lcd_print_str("Scan Finished!..");
                    lcd_print_str("...(^_^)  ");
                }

void ping_Distance(){

            lcd_clear();
            lcd_home();
            lcd_print_str("> IR-PSD Sensor");
            ambient_light = fetchDataADC(AMBIENT_LIGHT);                         // check ambient light conditions
            printString("ambient light = ");
            printWord(ired_range);                       // variables swapped (ired_range) bc of ADC update timing
            printString("\t");

            ired_range = fetchDataADC(IRED_SENSOR);                      // ping IRED-distance sensor for distance
            printString("\t\r\nSharp IRED-sensor ~ ping : ");
            printWord(ambient_light);                 // variables swapped (ambient_light) bc of ADC update timing

            distance = measureDistance(ambient_light);
            sprintf(distance_buffer, "%0.2f", distance);
            printString("\t distance(cm) = ");
            printString(distance_buffer);
            printString("\r\n\n");
}

void update_Telemetry(){

    lcd_clear();
    lcd_home();
    lcd_print_str("> telemetry mode");
//    lcd_home();
//    lcd_print_str("[Geo-Lab.] v.4.3");
//    lcd_print_str(" ");
//    _delay_ms(2000);
//    lcd_print_str("Physalis Labs.");
//    _delay_ms(1000);
//    lcd_clear();

    CTRLPWM_PORT ^= (1 << GREED_LED);                                             // toggle green LED every second

    ambient_light = fetchDataADC(AMBIENT_LIGHT);                                 // check ambient light conditions
    printString("ambient light = ");
    printWord(ired_range);                               // variables swapped (ired_range) bc of ADC update timing
    printString("\t");

    ired_range = fetchDataADC(IRED_SENSOR);                              // ping IRED-distance sensor for distance
    printString("\t\r\nSharp IRED-sensor ~ ping : ");
    printWord(ambient_light);                         // variables swapped (ambient_light) bc of ADC update timing

    distance = measureDistance(ambient_light);
    sprintf(distance_buffer, "%0.2f", distance);
    printString("\t distance(cm) = ");
    printString(distance_buffer);
    printString("\r\n\n");

    de_EnergizeStepper();                                                         // Stepping Motor Torque Release

    distanceTraveled(hall_S2R_pulse, hall_S2L_pulse);                   // Read Hall sensors for distance traveled

    printString("\t");

    printTime(year_, months_, days_, hours_, minutes_, seconds_);                  // Prints Date/Time to terminal

    printString("\r\n\n");
}
//------------------------------- Communication Functions ------------------------------------------------------//
void pcLinkSerial(void){                         // computer link!, function to shuttle data from robot to host PC

    char input;
    if((UCSR0A & (1 << RXC0))){
        input = UDR0;
        switch(input){
                case 'T' :
                    set_Time();
                        break;
                case 'O' :
                    set_Kinetics();
                        break;
                case 'W' :
                    roboDrive("backward", drive_speed);         // swapped forward << >> backward due to hardware
                        break;
                case 'S' :
                    roboDrive("forward", drive_speed);          // swapped forward << >> backward due to hardware
                        break;
                case 'A' :
                    roboDrive("left", drive_speed);
                        break;
                case 'D' :
                    roboDrive("right", drive_speed);
                        break;
                case 'X' :
                    roboDrive("brake", 0);
                    de_EnergizeStepper();
                        break;
                case '6' :
                    stepperDrive_adapted(stepper_speed, MIN_DELAY);
                        break;
                case '4' :
                    stepperDrive_adapted(-(stepper_speed), MIN_DELAY);
                        break;
                case '8' :
                    //roboDrive_LookUp(servo_speed_towerdrive);
                    roboDrive_LookUp_Gradually(38);
                        break;
                case '5' :
                    roboDrive_LookUpFront(servo_speed_towerdrive);
                        break;
                case '0' :
                    //roboDrive_LookDown(servo_speed_towerdrive);
                    roboDrive_LookDown_Gradually(38);
                        break;
                case '/' :
                    pcf8575_Output(0b1111111111111111);
                    printString("\r\nFront Light >> ON\n");
                        break;
                case '*' :
                   pcf8575_Output(0b0000000000000000);
                   printString("\r\nFront Light >> OFF\n");
                        break;
                case 'Y' :
                    //roboDrive_Arm_Testing();
                    roboDrive_DeployLeft_Arm_State4(servo_speed_mecha_arms);
                        break;
                case '#' :                     // apply only when both arms are streched, use this to recover from unknown states to parking
                    roboDrive_RecoverArmsForParking(servo_speed_mecha_arms);                     // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case '7' :                       // apply only when left arm is streched, use this to recover from unknown states to parking
                    roboDrive_RecoverLeftArmForParking(servo_speed_mecha_arms);                  // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case '9' :                      // apply only when right arm is streched, use this to recover from unknown states to parking
                    roboDrive_RecoverRightArmForParking(servo_speed_mecha_arms);                 // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'L' :                                      // ------------------- park arm (left) >> parked state (current draw ~ 0.6A)
                    roboDrive_ParkLeftArm(servo_speed_mecha_arms);                             // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'R' :                                      // ------------------ park arm (right) >> parked state (current draw ~ 0.6A)
                    roboDrive_ParkRightArm(servo_speed_mecha_arms);                            // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'a' :                                 // ------------------- deploy arm state-1 (left) >> state-1 (current draw ~ 0.6A)
                    roboDrive_DeployLeft_Arm_State1(servo_speed_mecha_arms);                   // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'l' :                                // ------------------- deploy arm state-1 (right) >> state-1 (current draw ~ 0.6A)
                    roboDrive_DeployRight_Arm_State1(servo_speed_mecha_arms);                  // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'k' :                                      // ------------- park arms (lef/right) >> parked state (current draw ~ 0.6A)
                    roboDrive_ParkArms(servo_speed_mecha_arms);                                // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case '1' :                              // ------------- deploy arms (state_1) (lef/right) >> state 1 (current draw ~ 0.75A)
                    roboDrive_DeployArmsState1(servo_speed_mecha_arms);                        // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case '2' :                         // ------------- deploy arms (state_2) (lef/right) >> state 2 (current draw ~ 0.50-0.65A)
                    roboDrive_DeployArmsState2(servo_speed_mecha_arms);                        // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case '3' :                         // ------------- deploy arms (state_3) (lef/right) >> state 3 (current draw ~ 0.60-0.68A)
                    roboDrive_DeployArmsState3(servo_speed_mecha_arms);                        // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case '-' :                                // ------------- activate arms ((lef/right) >> state 4 (current draw ~ 0.68-0.70A)
                    roboDrive_ActivateArms(servo_speed_mecha_arms);                            // speed >> ((superfast) 0 - 255 (very slow))
                          break;
                case '+' :                  // ------------- lift-up arms (^_^) show-off ((lef/right) >> state 4 (current draw ~ 0.68-0.70A)
                    roboDrive_LiftUpArms(servo_speed_mecha_arms);                              // speed >> ((superfast) 0 - 255 (very slow))
                          break;
                case 'd' :                                                    // ------------- gripper >> OPEN (left), (current draw ~ 0. A)
                    //roboDrive_OpenGripperLeftMechaArm(servo_speed_grippers);                 // speed >> ((superfast) 0 - 255 (very slow))
                    roboDrive_OpenLeftGripper_gradually(servo_speed_grippers);
                        break;
                case 'f' :                                                   // ------------- gripper >> OPEN (right), (current draw ~ 0. A)
                    //roboDrive_OpenGripperRightMechaArm(servo_speed_grippers);                // speed >> ((superfast) 0 - 255 (very slow))
                    roboDrive_CloseLeftGripper_gradually(servo_speed_grippers);
                        break;
                case 'h' :                                                    // ------------- gripper >> OPEN (left), (current draw ~ 0. A)
                    //roboDrive_OpenGripperLeftMechaArm(servo_speed_grippers);                 // speed >> ((superfast) 0 - 255 (very slow))
                    roboDrive_OpenRightGripper_gradually(servo_speed_grippers);
                        break;
                case 'j' :                                                   // ------------- gripper >> OPEN (right), (current draw ~ 0. A)
                    //roboDrive_OpenGripperRightMechaArm(servo_speed_grippers);                // speed >> ((superfast) 0 - 255 (very slow))
                    roboDrive_CloseRightGripper_gradually(servo_speed_grippers);
                        break;
                case 'C' :                                   // ------------- rotate CW [gripper] on RIGHT mech-arm >> (current draw ~ 0. A)
                     roboDrive_Rotate_CW_GripperLeftMechaArm(servo_speed_rotation_grippers);   // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'V' :                                                // rotate CCW [gripper] on RIGHT mech-arm >> (current draw ~ 0. A)
                    roboDrive_Rotate_CCW_GripperLeftMechaArm(servo_speed_rotation_grippers);   // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'N' :                                    // ------------- rotate CW [gripper] on LEFT mech-arm >> (current draw ~ 0. A)
                    roboDrive_Rotate_CW_GripperRightMechaArm(servo_speed_rotation_grippers);   // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'M' :                                   // ------------- rotate CCW [gripper] on LEFT mech-arm >> (current draw ~ 0. A)
                    roboDrive_Rotate_CCW_GripperRightMechaArm(servo_speed_rotation_grippers);  // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'P' :                                   // ------------- Probe Deploy [Left]  on LEFT mech-arm >> (current draw ~ 0. A)
                    roboDrive_Deploy_FlatSurfaceProbe(30);  // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'c' :                                          // ------------- [STEER >> CW] [RIGHT] mech-arm >> (current draw ~ 0. A)
                     roboDrive_move_LeftMechaArm_CW(servo_speed_turning_mecha_arms);           // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'v' :                                         // ------------- [STEER >> CCW] [RIGHT] mech-arm >> (current draw ~ 0. A)
                    roboDrive_move_LeftMechaArm_CCW(servo_speed_turning_mecha_arms);           // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'n' :                                           // ------------- [STEER >> CW] [LEFT] mech-arm >> (current draw ~ 0. A)
                    roboDrive_move_RightMechaArm_CW(servo_speed_turning_mecha_arms);           // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'm' :                                          // ------------- [STEER >> CCW] [LEFT] mech-arm >> (current draw ~ 0. A)
                    roboDrive_move_RightMechaArm_CCW(servo_speed_turning_mecha_arms);          // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'e' :                  // ------------- extends mecha-arm step by step forward >> extends (left), (current draw ~ 0. A)
                    roboDrive_move_Forward_LeftMechaArm(35);                                   // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'r' :                       // ------------- cranes mecha-arm step by step up >> Lifts-up (left), (current draw ~ 0. A)
                    roboDrive_Crane_LeftArm(35);                                               // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'o' :                 // ------------- extends mecha-arm step by step forward >> extends (right), (current draw ~ 0. A)
                    roboDrive_move_Forward_RightMechaArm(35);                                  // speed >> ((superfast) 0 - 255 (very slow))
                        break;
                case 'p' :                      // ------------- cranes mecha-arm step by step up >> Lifts-up (right), (current draw ~ 0. A)
                    roboDrive_Crane_RightArm(35);                                              // speed >> ((superfast) 0 - 255 (very slow))
                       break;
                case 'B' :
                    //vibration_Scanning();
                    roboDrive_ParkLeft_Arm_fromState4(servo_speed_mecha_arms);
                        break;
                case 'Z' :
                    temperatureScanner();
                        break;
                case 'F' :
                    read_FRAM();
                        break;
                case 'E' :
                    ping_Distance();
                        break;
                case 'Q' :
                    LIDAR_scanArea();
                    LIDAR_status = check_LIDAR_busyFlag();
                    if((LIDAR_status & LIDAR_LITE_BITMASK) == 0){
                            printString("distance = ");
                            LIDAR_distance = LIDAR_shuttle_Data();
                            printWord(LIDAR_distance);
                            printString("\r\n");
                    }
                    else{
                            printString("LIDAR is Busy....\r\n");
                            LIDAR_status = check_LIDAR_busyFlag();
                    }
                        break;
                case 'U' :
                    update_Telemetry();
                        break;
                case 'H' :
                    help_Readme();
                        break;
                default  :
                    printString("robo-drive: unrecognized command!...");
                    playNote(notes[15], currentNoteLength);
                    playNote(notes[16], currentNoteLength);
                    de_EnergizeStepper();
        }
    }
}

// void espLinkSilent(void){             // espLinkSilent() copy of espLinkSerial() with disabled debug printouts (telemetry clean)
//                                                                                    // packet format  = "~byte#byte#commandchar|"
// 	char thisChar = '0';
// 	uint8_t set_speed = 0;                                                         // robo-Drive() arguments for speed and rate
// 	uint8_t set_rate = 0;                                                          // robo-Drive() arguments for speed and rate
//
// 	do {
// 		thisChar = receiveByte();                                                        // get a new character, blocking loop!
// 		if (thisChar == 'S') {
//                                 roboDrive("brake", 0);
//                                 de_EnergizeStepper();
// 							}
//
//   } while (thisChar != '~');
// 		thisChar = receiveByte();
// 		set_speed = thisChar;
// 		thisChar = receiveByte();
// 		thisChar = receiveByte();
// 		set_rate = thisChar;
// 		thisChar = receiveByte();
// 		thisChar = receiveByte();
//
//         switch(thisChar){
//                 case 'F' :
// 						roboDrive("forward", set_speed);
//                         break;
//                 case 'B' :
// 		                  roboDrive("backward", set_speed);
//                         break;
//                 case 'L' :
// 						roboDrive("left", set_speed);
//                         break;
//                 case 'R' :
// 						roboDrive("right", set_speed);
//                         break;
//                 case 'S' :
//                     roboDrive("brake", 0);
//                     de_EnergizeStepper();
//                         break;
//                 case 'A' :
//                     stepperDrive_adapted(-3, 200);                      // delay set to constant value (200), not from joystick
//                         break;
//                 case 'D' :
//                     stepperDrive_adapted( 3, 200);                      // delay set to constant value (200), not from joystick
//                         break;
//                 default  :
//                     printString("robo-drive: unrecognized command!...\r\n");
//                     playNote(notes[0], currentNoteLength);
//                     playNote(notes[1], currentNoteLength);
//                     playNote(notes[6], currentNoteLength);
//         }
// 		thisChar = receiveByte();				                                                         // get a new character
// 		thisChar = receiveByte();				                                                         // get a new character
// 		if (thisChar == 'S') {
//                                 roboDrive("brake", 0);
//                                 de_EnergizeStepper();
// 							}
// }
//
// void espLinkSerialTest(void){                            // espLinkSerialTest() used to view codes of bytes from Remote Control
//
// 	char thisChar = '0';                                                                             // *byte#byte#commandchar|
//
// 	do {
// 		thisChar = receiveByte();                                                                        // get a new character
// 		//transmitByte(thisChar);                                                                                       // echo
// 		printByte(thisChar);
// 		transmitByte(':');
//
//  } while (thisChar != '+');
// }
//
// void espLinkSilent_v3(void){             	// v3_Telemetry - added telemetry and lights control bytes
// 											// [~][speed-byte][telemetry-byte][rotation-byte][lights-byte][1st-commandchar-tracks][2nd-commandchar-tower][|]
// 	char thisChar = '0';
// 	uint8_t set_speed = 0;                                                         // robo-Drive() arguments for speed and rate
// 	uint8_t set_rate = 0;                                                          // robo-Drive() arguments for speed and rate
// 	uint8_t set_lights = 0;                                                        // arguments for lights control
//
// 	do {
// 		thisChar = receiveByte();            // get a new character, blocking loop!
// 		if (thisChar == 'S') {
//                                 roboDrive("brake", 0);
//                                 de_EnergizeStepper();
// 							}
//
//   } while (thisChar != '~');
// 		thisChar = receiveByte();			// receive speed
// 		set_speed = thisChar;
// 		thisChar = receiveByte();			// receive telemetry control char
//
//
//     if (thisChar != '0') {
//         reportUART_12bytes(thisChar);		// call telemetry reporter
//         }
// 	//	reportUART_12bytes(thisChar);		// call telemetry reporter
//
// 		thisChar = receiveByte();			// receive rotation rate
// 		set_rate = thisChar;
// 		thisChar = receiveByte();			// receive lights control byte
// 		set_lights = thisChar;
// 											// call lights control function
// 		thisChar = receiveByte();			// receive first command char
//
//         switch(thisChar){
//                 case 'F' :
// 						roboDrive("forward", set_speed);
//                         break;
//                 case 'B' :
// 		                roboDrive("backward", set_speed);
//                         break;
//                 case 'L' :
// 						roboDrive("left", set_speed);
//                         break;
//                 case 'R' :
// 						roboDrive("right", set_speed);
//                         break;
// 				case 'N' :
// 						roboDrive("brake", 0);
// 						break;
//                 case 'S' :
// 						roboDrive("brake", 0);
// 						de_EnergizeStepper();
//                         break;
//                 default  :
// //						printString("robo-drive: unrecognized command!...\r\n");
// 						playNote(notes[0], currentNoteLength);
// 						playNote(notes[1], currentNoteLength);
// 						playNote(notes[6], currentNoteLength);
//         }
//
// 		thisChar = receiveByte();			// receive second command char
//
//         switch(thisChar){
//                 case 'A' :
// 						stepperDrive_adapted(-3, 200);                      // delay set to constant value (200), not from joystick
//                         break;
//                 case 'D' :
// 						stepperDrive_adapted( 3, 200);                      // delay set to constant value (200), not from joystick
//                         break;
// 				case 'N' :
// 						de_EnergizeStepper();
// 						break;
// 				case 'S' :
// 						roboDrive("brake", 0);
// 						de_EnergizeStepper();
//                         break;
//                 default  :
//                     // printString("robo-drive: unrecognized command!...\r\n");
//                     playNote(notes[0], currentNoteLength);
//                     playNote(notes[0], currentNoteLength);
//                     playNote(notes[0], currentNoteLength);
//         }
//
//
// 		thisChar = receiveByte();				                                                         // get a new character
// 		thisChar = receiveByte();				                                                         // get a new character
// 		if (thisChar == 'S') {
//                                 roboDrive("brake", 0);
//                                 de_EnergizeStepper();
// 							}
// }
// ----------------------------------------- ISR() Interrupt Functions -------------------------------------------------------//
ISR(TIMER1_OVF_vect){                                                                   // system ticks tee-hee tee-hee ... ^_^)

    sys_tick ++;
}

ISR(TIMER0_COMPA_vect){                                                // Timer/Counter-0 Compare match interrupt vector enables

        stepPhase += direction;                                                                  // take step in right direction
        stepPhase &= 0b00000111;                                                          // keep the stepPhase in range (0 - 7)
        STEPPER_PORT = motor_Phases[stepPhase];                                           // write phase out to motor COIL-1 A/B
        HALL_PORT = motor_Phases[stepPhase];                                              // write phase out to motor COIL-2 A/B
        stepCounter ++;                                                                                   // keep track of steps
}

ISR(PCINT2_vect){                                                               // sub - ISR() monitoring Hall sensors on motors

    telemetryMotorRight();
    telemetryMotorLeft();
}

EMPTY_INTERRUPT(ADC_vect);                                            // fake interrupt to wake the CPU.... does not do anything

// ISR(USART0_RX_vect){
//
//     printString("\r\n >> USART RX Receive Interrupt Enabled!\r\n");
//     pcLinkSerial();
// }
//----------------------------------------- Initializations ------------------------------------------------------------------//
int main(void){

        initUSART();                                                                     // USART hardware initialize and enable
        printString("\r\n >> roboDrive: starting >>UART\r\n");
        initSPI();                                                                                       // enable SPI interface
        printString("\r\n >> roboDrive: starting >>SPI\r\n");
        initI2C();                                                                  // enable inter-integrated circuit interface
        printString("\r\n >> roboDrive: starting >>I-2C\r\n");
        lcd_init();                                                                                   // enable LCD-display link
        printString("\r\n >> roboDrive: starting >>LCD\r\n");
        initTimer_0();                                                         // Timer/Counter-0 hardware initialize and enable
        printString("\r\n >> roboDrive: starting >>TIMER-0\r\n");
        initTimer_1();                                                         // Timer/Counter-1 hardware initialize and enable
        printString("\r\n >> roboDrive: starting >>TIMER-1\r\n");
        initTimer_2();                                                         // Timer/Counter-2 hardware initialize and enable
        printString("\r\n >> roboDrive: starting >>TIMER-2\r\n");
        initTimer_3();                                                         // Timer/Counter-3 hardware initialize and enable
        printString("\r\n >> roboDrive: starting >>TIMER-3\r\n");
        initADC();                                                                               // On-board ADC hardware enable
        printString("\r\n >> roboDrive: starting >>ADC\r\n");
        initGPIOchangeInterrupts();                                                              // GPIO Interrupt system enable
        printString("\r\n >> roboDrive: starting >>ISRs\r\n");
        setup_ADC_sleepmode();                                     // only ADC hardware runs, CPU sleeps during the measurements

        DIDR0 |= (1 << ADC4D) | (1 << ADC5D) | (1 << ADC6D) | (1 << ADC7D);  // disable digital functions when,  analog sampling
        HALL_PORT |= ((1 << HALL_S1R) | (1 << HALL_S2R) | (1 << HALL_S1L) | (1 << HALL_S2L)); // pull-ups on Hall sensors enable
        CTRL_DDR |= (1 << RBN3_FET) | (1 << RBN4_FET);                  // left BRIDGE low-side data direction setup output mode
        CTRL_DDR |= (1 << LBN7_FET) | (1 << LBN8_FET);                 // right BRIDGE low-side data direction setup output mode
        STEPPER_DDR |= (1 << COIL_A1) | (1 << COIL_A2);                  // stepper COIL-A GPIO data direction setup output mode
        HALL_DDR |= (1 << COIL_B1) | (1 << COIL_B2);                     // stepper COIL-B GPIO data direction setup output mode
        STEPPER_DDR |= (1 << SPEAKER);                                   // system speaker GPIO data direction setup output mode
        CTRLPWM_DDR |= (1 << GREED_LED);                              // system status LED GPIO data direction setup output mode
        //CTRLPWM_DDR |= (1 << RED_LED_1);                           // vibration sens. LED1 GPIO data direction setup output mode
        //HALL_DDR |= (1 << RED_LED_2) | (1 << RED_LED_3);      // vibration sens. LED2/LED3 GPIO data direction setup output mode
        sei();                                                                                      // global interrupt flag set
//        for(int i = 0; i < 11; i ++){
//            playNote(notes[i+1], currentNoteLength);
//            _delay_ms(200);
//        }
//        playNote(notes[15], currentNoteLength);                              // sound tones played during system initialization
//        _delay_ms(200);
//        playNote(notes[4], currentNoteLength);
//        _delay_ms(150);
//        playNote(notes[4], currentNoteLength);
//        _delay_ms(150);
//        playNote(notes[4], currentNoteLength);
//        _delay_ms(150);
//        playNote(notes[15], currentNoteLength);
//        _delay_ms(150);
//        playNote(notes[15], currentNoteLength);
//        _delay_ms(150);
//        playNote(notes[16], currentNoteLength);
//        _delay_ms(200);
//        playNote(notes[15], currentNoteLength);
        printString("\r\n\n -------  [Kenji-X1] / Firmware Version 5.8 \r\n\n");                            // version control
        printString("   -------  Compiled on: " __DATE__" / "__TIME__" \r\n\n");                        // Build Date and Time
        printString("   -------  Platform Status: [in development] ... \r\n\n");                    // Platform current status
        //printString("\r\n    -------  > roboDrive Instruction set <-------\r\n");                      // roboDrive OP-codes
        printString("\r\n\n ---> [roboDrive Engine Instruction set] <---\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n   -------  > Run Control < ----------------------\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n -> press [T] (Update Time/Date)\r\n");
        printString("\r\n -> press [O] (Configure Kinetics and Parameters)\r\n");
        printString("\r\n -> press [B] (Start Vibrations Scanning)\r\n");
        printString("\r\n -> press [E] (Start >[IR]< Distance Measurement)\r\n");
        printString("\r\n -> press [Q] (Start >[LASER]< Distance Scan on LIDAR)\r\n");
        printString("\r\n -> press [Z] (Start a Temperature Scan)\r\n");
        printString("\r\n -> press [U] (View Telemetry)\r\n");
        printString("\r\n -> press [F] (Memory Read/Write Operations)\r\n");
        printString("\r\n -> press [H] (Help/Readme)\r\n\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n   -------  > Drivetrain Control < ----------------------\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n -> press [W] (Forward Drive)\r\n");
        printString("\r\n -> press [S] (Reverse Drive/Go-Back)\r\n");
        printString("\r\n -> press [A] (Turn Left)\r\n");
        printString("\r\n -> press [D] (Turn Right)\r\n");
        printString("\r\n -> press [X] (Stop|Brake)\r\n\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n   -------  > Tower Control < ----------------------------\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n -> press [4] (Look Left)\r\n");
        printString("\r\n -> press [6] (Look Right)\r\n");
        printString("\r\n -> press [8] (Look Up)\r\n");
        printString("\r\n -> press [5] (Look Up-Front)\r\n");
        printString("\r\n -> press [0] (Look Down)\r\n");
        printString("\r\n -> press [SHIFT]+[*] (Light OFF)\r\n");
        printString("\r\n -> press [SHIFT]+[/] (Light ON)\r\n\n");
        printString("\r\n -> cycle [SHIFT]+[/] with [SHIFT]+[*] (Mode Change)\r\n\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n   -------  > Manipulator Control < ----------------------\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n -> press [#] (Recover Both Arms from random position)\r\n");
        printString("\r\n -> press [7] (Recover Left-Arm  from random position (arm has been touched/moved by accident in OFF state))\r\n");
        printString("\r\n -> press [9] (Recover Right-Arm from random position (arm has been touched/moved by accident in OFF state))\r\n");
        printString("\r\n -> press [Y] (Custom Servo Commands)\r\n");
        printString("\r\n -> press [k] (Park Both Arms)\r\n");
        printString("\r\n -> press [L] (Park Left  Mecha-Arm)\r\n");
        printString("\r\n -> press [R] (Park Right Mecha-Arm)\r\n");
        printString("\r\n -> press [a] (Deploy State-1 Left  Mecha-Arm)\r\n");
        printString("\r\n -> press [l] (Deploy State-1 Right Mecha-Arm)\r\n");
        printString("\r\n -> press [SHIFT]+[-] (Activate Both Arms)\r\n");
        printString("\r\n -> press [SHIFT]+[+] (Lift-Up Both Arms)\r\n\n");
        printString("\r\n -> press [1] (Deploy Both Arms > State-1)\r\n");
        printString("\r\n -> press [2] (Deploy Both Arms > State-2)\r\n");
        printString("\r\n -> press [3] (Deploy Both Arms > State-3)\r\n");
        printString("\r\n -> press [c] (Steer Left Mecha-Arm CW)\r\n");
        printString("\r\n -> press [v] (Steer Left Mecha-Arm CCW)\r\n");
        printString("\r\n -> press [n] (Steer Right Mecha-Arm CW)\r\n");
        printString("\r\n -> press [m] (Steer Right Mecha-Arm CCW)\r\n");
        printString("\r\n -> press [e] (Extend Forward >> Gradually Left  Mecha-Arm )\r\n");
        printString("\r\n -> press [r] (Crane Backward >> Gradually Left  Mecha-Arm )\r\n");
        printString("\r\n -> press [o] (Extend Forward >> Gradually Right Mecha-Arm )\r\n");
        printString("\r\n -> press [p] (Crane Backward >> Gradually Right Mecha-Arm )\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n   -------  > Gripper Control < --------------------------\r\n");
        printString("\r\n   --------------------------------------------\r\n");
        printString("\r\n -> press [d] (Open  Gradually >> Left  Gripper)\r\n");
        printString("\r\n -> press [f] (Close Gradually >> Left  Gripper)\r\n");
        printString("\r\n -> press [h] (Open  Gradually >> Right Gripper)\r\n");
        printString("\r\n -> press [j] (Close Gradually >> Right Gripper)\r\n");
        printString("\r\n -> press [C] (Rotate Left  Mecha-Arm Gripper CW )\r\n");
        printString("\r\n -> press [V] (Rotate Left  Mecha-Arm Gripper CCW)\r\n");
        printString("\r\n -> press [N] (Rotate Right Mecha-Arm Gripper CW )\r\n");
        printString("\r\n -> press [M] (Rotate Right Mecha-Arm Gripper CCW)\r\n");
        printString("\r\n   -------------------------------------------------------\r\n");
        printString("\r\n\n System Ready >> \r\n");
        i2cStart();
        lcd_init();
        for(int psi = 0 ; psi < 10; psi ++)
            {
                lcd_home();
                lcd_print_str(" Physalis Labs  ");
                lcd_print_str("roboDrive v_5.8");
                pcf8575_Output(0b0000000000000000);    // I2C port expander's Ports(0-7) as input, Ports(10-17) as output
                playNote(notes[psi], currentNoteLength);
                _delay_ms(50);
        }
        lcd_clear();
        for(int psi = 10 ; psi > 0; psi --)
            {
                lcd_home();
                lcd_print_str("Waking up (^_^)");
                pcf8575_Output(0b1111111111111111);    // I2C port expander's Ports(0-7) as input, Ports(10-17) as output
                playNote(notes[psi], currentNoteLength);
                _delay_ms(10);
        }
        lcd_clear();
        for(int psi = 0 ; psi < 12; psi ++)
            {
                lcd_home();
                lcd_print_str("please update _ ");
                lcd_print_str("> kinetics >> |O|");
                pcf8575_Output(0b0000000000000000);    // I2C port expander's Ports(0-7) as input, Ports(10-17) as output
                playNote(notes[1], currentNoteLength);
                _delay_ms(100);
                playNote(notes[3], currentNoteLength);
                pcf8575_Output(0b1111111111111111);
        }
        lcd_clear();
        i2cStop();
        for(int i = 80; i < 190; i=i+1){
            runServo_onPCA9685(0, i);
            _delay_ms(7);
        }
        initPCA9685(PCA9685_ADDRESS_W, 50);
        move_Servo_Bidirect(14, 10, 50, 1);                      // activating Tower Servo-Drive Unit to keep it on place
//-------------------------------------- Event loop -------------------------------------------------------------------//
        while(1){
            if(sys_tick == OVERFLOWS_PER_SECOND){
                sys_tick = 0;
                secondsRun();
                //millisecondsRun();
            }
            pcLinkSerial();
            //espLinkSilent();
            //espLinkSerialTest();
            //espLinkSilent_v2();
            //espLinkSilent_v3();
        }
    return(0);
}
