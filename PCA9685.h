#ifndef PCA9685_H_INCLUDED
#define PCA9685_H_INCLUDED

// -------------- from datasheet, the PCA9685 OP-codes ---------------------//

#define PCA9685_ADDRESS_W           0b10100000  // A4 = 1, A0-A5 = 0, 0-write bit (i2C)
#define PCA9685_ADDRESS_R           0b10100001  // A4 = 1, A0-A5 = 0, 1-read bit  (i2C)

#define PCA9685_SUBADR1             0x2
#define PCA9685_SUBADR2             0x3
#define PCA9685_SUBADR3             0x4

#define PCA9685_MODE1               0x0
#define PCA9685_MODE2               0x1

#define PCA9685_PRESCALE            0xFE

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
/*
 * Configure PCA9685 slave device
 *
 * Sets the slave device MODE1, PRESCALE (controls the output frequency) and
 * MODE2 register values.
 *
 * Arguments:
 *      address    The PCA9685 hardwired 6-bit slave device address.
 *
 *      freq       Output driver frequency (in Hz). Must be a value between
 *                 24 Hz and 1526 Hz
 */
void pca9685_init(uint16_t freq);
/*
 * Set the servo horn angle
 *
 * Sets the slave device MODE1, PRESCALE (controls the output frequency) and
 * MODE2 register values.
 *
 * Arguments:
 *      servoNum    The index of the servo motor. A number between 0 and 15.
 *
 *      angle       Angle in degrees. Must be a value between -90 and 90
 *                  degrees.
 */
void pca9685_servo(uint8_t servoNum, float angle);

#endif // PCA9685_H_INCLUDED
