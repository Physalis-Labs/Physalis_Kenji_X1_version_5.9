// -------------- 16-channel, 12-bit PWM Fm+ I2C-bus LED controller ----------------//
/*16 LED drivers. Each output programmable at:
 Off
 On
 Programmable LED brightness
 Programmable LED turn-on time to help reduce EMI*/

#ifndef PCA9685_PWM_CONTROLLER_H_INCLUDED
#define PCA9685_PWM_CONTROLLER_H_INCLUDED

// -------------- Device OP-codes, from datasheet ----------------------------------//

#define PCA9685_ADDRESS_W           0b10100000  // A4 = 1, A0-A5 = 0, 0-write bit (i2C)
#define PCA9685_ADDRESS_R           0b10100001  // A4 = 1, A0-A5 = 0, 1-read bit  (i2C)

#define PCA9685_SUBADR1             0b00000010
#define PCA9685_SUBADR2             0b00000011
#define PCA9685_SUBADR3             0b00000100

#define PCA9685_MODE1               0b00000000
#define PCA9685_MODE2               0b00000001

#define PCA9685_PRESCALE            0xFE        // Prescaler register which stores the PWM frequency aka update rate

#define LED0_ON_L                   0x6
#define LED0_ON_H                   0x7
#define LED0_OFF_L                  0x8
#define LED0_OFF_H                  0x9

#define ALLLED_ON_L                 0xFA
#define ALLLED_ON_H                 0xFB
#define ALLLED_OFF_L                0xFC
#define ALLLED_OFF_H                0xFD

#define SERVO0                      0x06

#define NEUTRAL_PULSE               1500
#define MAX_PULSE                   2100
#define MIN_PULSE                   900
#define MAX_ANGLE                   90

/** \brief function initializes the PCA-9685 device and configures the operation
 *         according to arguments.
 * \param  i2c_address, device i2C address on BUS
 * \param  the PWM update frequency, for operation of the controller
 * \return does not return anything
 */
void initPCA9685(uint8_t i2c_address, uint16_t controller_freq);

/** \brief function is meant to drive any given servo-motor, to a certain degrees,
 *         using the PCA-9685 controller on the i2C BUS. It can control several servos simultaneously.
 * \param  servo_number, up to 16 channels.
 * \param  degree, signed integer, how many degrees in which direction to move.
 * \return does not return anything
 */
void runServo_onPCA9685(uint8_t servo_number, float angle);

#endif // PCA9685_PWM_CONTROLLER_H_INCLUDED
