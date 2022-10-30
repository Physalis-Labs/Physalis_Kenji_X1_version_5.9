//-------------- 16-channel, 12-bit PWM Fm+ I2C-bus LED controller ----------------//
/* 16 LED drivers. Each output programmable at:
 - Off
 - On
 - Programmable LED brightness
 - Programmable LED turn-on time to help reduce EMI*/

#include <avr/io.h>
#include <util/twi.h>
#include <i2c.h>
#include <util/delay.h>
#include <PCA9685_PWM_Controller.h>
#include <USART.h>

//-------------- initialize PCA9685 device, mode setup and start ------------------//

uint16_t global_freq_pwm;                                         // this variable shares the frequency value among the two function, makes it visible aka global
uint8_t global_address;                                                   // this variable shares the address among the two function, makes it visible aka global

void initPCA9685(uint8_t i2c_address, uint16_t controller_freq){                        // update rate, PWM frequency on pins, takes values in range(24Hz-1525Hz)

    //uint8_t session_code;                                                                                                                 // from AVR generated
    volatile uint8_t prescalar;                                                                                        // prescaler for PWM frequency calculation
    global_freq_pwm = controller_freq;                                             // update the global variables with the frequency and address from arguments provided
    global_address = i2c_address;
    i2cStart();                                                                                                                     // start a i2C session on BUS
    i2cSend(i2c_address);                                                                        // send the PCA9685 device address with write 0-Bit, 0xA0 in hex
//    session_code = TWSR;                                                                // read the AVR generated i2C transaction code into variable session code
//    if(session_code == TW_MT_SLA_ACK){                                                                    // if PCA9685 responded with "Ack" proceed to next step
//        printString("PCA9685 address sent!, device accessible\r\n");
//    }
//    else{
//        printString("Nack received! ending session-1\r\n");
//        i2cStop();
//    }
    i2cSend(PCA9685_MODE1);                                                                                     // send address of the Mode-1 register, datasheet
//    session_code = TWSR;                                                                                           // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                   // if PCA9685 responded with "Ack" proceed to next step
//        printString("Mode-1 control register address sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-2\r\n");
//        i2cStop();
//    }                                                                                       // setup register configuration, select slave, select MODE1 register,
    i2cSend(0b00110001);                                                                 // then set AI (auto-increment) enable, SLEEP active, and ALLCALL enable
//    session_code = TWSR;                                                                                           // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                   // if PCA9685 responded with "Ack" proceed to next step
//        printString("Mode-1 control register config. sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-3\r\n");
//        i2cStop();
//    }
    i2cStop();
    prescalar = (25000000 / ((float)4096 * controller_freq * 0.92)) - 1;
//prescalar = round(25000000 / (4096 * pwm_pulse_freq)) - 1;                              // see datasheet page.25 // PWM frequency and prescaler relationship eq.
// ------------------- prescaler register -----------------------------------------//
    i2cStart();                                                                                                               // start a i2C session with PCA9685
    i2cSend(i2c_address);                                                                        // send the PCA9685 device address with write 0-Bit, 0xA0 in hex
//    session_code = TWSR;                                                                                           // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_SLA_ACK){                                                                    // if PCA9685 responded with "Ack" proceed to next step
//        printString("PCA9685 address sent!, device accessible\r\n");
//    }
//    else{
//        printString("Nack received! ending session-4\r\n");
//        i2cStop();
//    }
    i2cSend(PCA9685_PRESCALE);                                                                         // send the PCA9685 prescale register address, 0xFE in hex
//    session_code = TWSR;                                                                                           // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                   // if PCA9685 responded with "Ack" proceed to next step
//        printString("PCA9685 prescaler register address sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-5\r\n");
//        i2cStop();
//    }
    i2cSend(prescalar);                                                                                                 // write the prescaler value to register
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                  // if PCA9685 responded with "Ack" proceed to next step
//        printString("PCA9685 prescaler saved!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-6\r\n");
//        i2cStop();
//    }
    i2cStop();
    _delay_ms(1);
// ------------------------------------ change settings in Mode-1 register ------------------------------------------//
    i2cStart();                                                                                                                    // start a i2C session on BUS
    i2cSend(i2c_address);                                                                       // send the PCA9685 device address with write 1-Bit, 0x40 in hex
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_SLA_ACK){                                                                   // if PCA9685 responded with "Ack" proceed to next step
//        printString("PCA9685 address sent!, device accessible\r\n");
//    }
//    else{
//        printString("Nack received! ending session-7\r\n");
//        i2cStop();
//    }
    i2cSend(PCA9685_MODE1);                                                                                    // send address of the Mode-1 register, datasheet
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                  // if PCA9685 responded with "Ack" proceed to next step
//        printString("Mode-1 control register address sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-8\r\n");
//        i2cStop();
//    }
    i2cSend(0b10100001);                                                                           // changing the register configuration, select MODE1 register,
                                                                            // then set AI(auto-increment) enable, SLEEP disable, ALLCALL enable, RESTART enable
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                  // if PCA9685 responded with "Ack" proceed to next step
//        printString("Mode-1 control register config. sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-9\r\n");
//        i2cStop();
//    }
    i2cStop();
    _delay_ms(1);
//------------------------------------------ Mode-2 register setup -------------------------------------------------//
    i2cStart();                                                                                                                    // start a i2C session on BUS
    i2cSend(i2c_address);                                                                       // send the PCA9685 device address with write 0-Bit, 0xA0 in hex
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_SLA_ACK){                                                                   // if PCA9685 responded with "Ack" proceed to next step
//        printString("PCA9685 address sent!, device accessible\r\n");
//    }
//    else{
//        printString("Nack received! ending session-10\r\n");
//        i2cStop();
//    }
    i2cSend(PCA9685_MODE2);                                                                                    // send address of the Mode-2 register, datasheet
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                  // if PCA9685 responded with "Ack" proceed to next step
//        printString("Mode-2 control register address sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-11\r\n");
//        i2cStop();
//    }
    i2cSend(0b00000100);                                                                /* Select slave device, select MODE2 register, then set INVRT (inverted
                                                                                           output) disable, OCH (outputs change on STOP command) enable, OUTDRV
                                                                                        (output driver configuration): totem pole output, and OUTNE:(output not
                                                                                                                                        enable mode) to 0x00 */

//    session_code = TWSR;                                                                                         // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_DATA_ACK){                                                                 // if PCA9685 responded with "Ack" proceed to next step
//        printString("Mode-2 control register config. sent!\r\n");
//    }
//    else{
//        printString("Nack received! ending session-12\r\n");
//        i2cStop();
//    }
    i2cStop();
}
//------------------------------------------- Mode-2 register setup ends here---------------------------------------//
void runServo_onPCA9685(uint8_t servo_number, float angle){

    uint16_t pulse_us;
    uint16_t period_us;
    uint16_t count;
    uint8_t off_command_highbyte;
    uint8_t off_command_lowbyte;
    //uint8_t session_code;

    if(angle > 125){            //if this filter is disabled, int8_t will overflow and reach unsafe values
        angle = 125;
    }
    else if(angle < - 125){
        angle = -125;
    }
    else{}
                                                                                        // Calculate pulse duration in us (note: equation can be optimized)
    pulse_us = NEUTRAL_PULSE + angle * ((float)(MAX_PULSE - MIN_PULSE) / (2 * MAX_ANGLE));
                                                                                            /* Convert pulse duration into a value from 0 to 4096, which will be
                                                                                                                  repeated every (1 / frequency) * 1000000 us */
    period_us = (float)1000000 / global_freq_pwm;
                                                                                      /* Output turns on at 0 counts (simplest way), and will turn off according
                                                                                          to calculations above. Break the 12-bit count into two 8-bit values */
    count = ((float)pulse_us / period_us) * 4096;
    off_command_lowbyte = count;
    off_command_highbyte  = count >> 8;
                                                                                    /* Each output is controlled by 2x 12-bit registers: ON to specify the count
                                                                                          time to turn on the LED (a number from 0-4095), and OFF to specify the
                                                                                              count time to turn off the LED (a number from 0-4095). Each 12-bit
                                                                                                   register is composed of 2 8-bit registers: a high and low. */

                                                                                             /* Select slave device, select LEDXX_ON_L register, set contents of
                                                                                          LEDXX_ON_L, then set contents of next 3 registers in sequence (only if
                                                                                                                                  auto-increment is enabled). */
    i2cStart();                                                                                                                    // start a i2C session on BUS
    i2cSend(global_address);                                                                    // send the PCA9685 device address with write 0-Bit, 0xA0 in hex
//    session_code = TWSR;                                                                                          // read the AVR generated i2C transaction code
//    if(session_code == TW_MT_SLA_ACK){                                                                   // if PCA9685 responded with "Ack" proceed to next step
//            printString("PCA9685 address sent!, device accessible\r\n");
//    }
//    else{
//            printString("Nack received! ending session-13\r\n");
//            i2cStop();
//    }
    i2cSend(SERVO0 + (4 * servo_number));                                                                                          // select LEDXX_ON_L register
    i2cSend(0x00);                                                                                                                    // set value of LEDXX_ON_L
    i2cSend(0x00);                                                                                                                    // set value of LEDXX_ON_H
    i2cSend(off_command_lowbyte);
    i2cSend(off_command_highbyte);
    i2cStop();
}
