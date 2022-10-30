/*--------------------------------------------------------------------------------------
- Garmin LIDAR-LITE v.3HP Drivers,                                                     -
- Operating temperature: -20° to 60°C (-4 to 140°F)                                    -
- Power:5 Vdc nominal, 4.5 Vdc min., 5.5 Vdc max.                                      -
- Current consumption: 65mA idle, 85mA during an acquisition                           -
- Range: (70% reflective target) - 40 m (131 ft.)                                      -
- Resolution: ±1 cm (0.4 in.)                                                          -
- Accuracy: < 2m ±5cm (2 in.) typical, NOTE: Nonlinearity present below 1 m (39.4 in.) -
- --------- Laser Specifications ----------------------------------------------------- -
- Wavelength: 905 nm (nominal)                                                         -
- Total laser power (peak): 1.3 W                                                      -
- Pulse width: 0.5 us (50% duty cycle)                                                 -
- Pulse train repetition frequency: 10-20 kHz nominal                                  -
- Energy per pulse: <280 nJ                                                            -
- Beam diameter at laser aperture: 12 × 2 mm (0.47 × 0.08 in.)                         -
- Divergence: 8 mRad                                                                   -
- -------- Interface Specifications -------------------------------------------------- -
- i2C interface: 400 kHz frequency                                                     -
- PWM interface: available                                                             -
- ------------------------------------------------------------------------------------ -
- written by Dr.-Ing. David Zohrabyan, Potsdam 17.02.2021 at Physalis Labs.            -

- start by doing what is necessary; then do what is possible; and suddenly,            -
- you are doing the impossible.                                                        -
--------------------------------------------------------------------------------------*/

#ifndef GARMIN_LIDAR_LITE_V3HP_H_INCLUDED
#define GARMIN_LIDAR_LITE_V3HP_H_INCLUDED

#include <avr/io.h>
#include <Hardware_Bay.h>
#include <i2c.h>


#define LIDAR_LITE_ADDRESS_W         0b11000100 // LIDAR address with wire bit
#define LIDAR_LITE_ADDRESS_R         0b11000101 // LIDAR address with read bit
#define LIDAR_LITE_STATUS_REGISTER   0b00000001 // LIDAR status register address
#define LIDAR_LITE_BITMASK           0b00000001 // to check if the LSB of the register is 1 or zero
//--------------------- Control Register List-------------------------------------------------------//
//---------- NOTE: Unless otherwise noted, all registers contain one byte and are read and write ---//
//---------- GARMIN LIDAR-LITE v3HP control register instructions and hex codes, from datasheet ----//
//---------- Datasheet version: November 2018 190-02088-02_0B --------------------------------------//

#define LIDAR_LITE_ACQ_COMMAND              0x00  // Writable,  Device Command, no initial value
#define LIDAR_LITE_STATUS                   0x01  // Read only, System Status, no initial value
#define LIDAR_LITE_SIG_COUNT_VAL            0x02  // Read/Write, Maximum acquisition count, initial value 0xFF
#define LIDAR_LITE_ACQ_CONFIG_REG           0x04  // Read/Write, Acquisition mode control, initial value 0x08
#define LIDAR_LITE_LEGACY_RESET_EN          0x06  // Writable,  Enable Unit Reset, no initial value
#define LIDAR_LITE_SIGNAL_STRENGTH          0x0e  // Read only, Received signal strength, no initial value
#define LIDAR_LITE_FULL_DELAY_HIGH          0x0f  // Read only, Distance measurement high byte, no initial value
#define LIDAR_LITE_FULL_DELAY_LOW           0x10  // Read only, Distance measurement low byte, no initial value
#define LIDAR_LITE_REF_COUNT_VAL            0x12  // Read/Write, Reference acquisition count, initial value 0x03
#define LIDAR_LITE_UNIT_ID_HIGH             0x16  // Read only, Serial number high byte, unique value
#define LIDAR_LITE_UNIT_ID_LOW              0x17  // Read only, Serial number low byte, unique value
#define LIDAR_LITE_I2C_ID_HIGH              0x18  // Writable, Write serial number high byte for I2C address unlock, no initial value
#define LIDAR_LITE_I2C_ID_LOW               0x19  // Writable, Write serial number low byte for I2C address unlock, no initial value
#define LIDAR_LITE_I2C_SEC_ADDR             0x1a  // Writable, Write new I2C address after unlock, no initial value
#define LIDAR_LITE_THRESHOLD_BYPASS         0x1c  // Read/Write, Peak detection threshold bypass, initial value 0x00
#define LIDAR_LITE_I2C_CONFIG               0x1e  // Read/Write, Default address response control, initial value 0x00
#define LIDAR_LITE_PEAK_STACK_HIGH_BYTE     0x26  // Read/Write, Used for post processing of correlation peak data, no initial value
#define LIDAR_LITE_PEAK_STACK_LOW_BYTE      0x27  // Read/Write, Used for post processing of correlation peak data, no initial value
#define LIDAR_LITE_COMMAND                  0x40  // Read/Write, State command, no initial value
#define LIDAR_LITE_HEALTH_STATUS            0x48  // Read only, Used to diagnose major hardware issues at initialization, no initial value
#define LIDAR_LITE_CORR_DATA                0x52  // Read only, Correlation record data low byte, no initial value

#define LIDAR_LITE_CORR_DATA_SIGN           0x53 // Read only, Correlation record data high byte, no initial value
#define LIDAR_LITE_POWER_CONTROL            0x65 // Read/Write, Read only, Power state control, initial value 0


void LIDAR_scanArea(void); // function to setup LIDAR to start measuring.

uint8_t check_LIDAR_busyFlag(void); // function to poll the LIDAR on-board status register for status / 0 free, 1 busy.

uint16_t LIDAR_shuttle_Data(void);  // function to get LIDAR measurement data, returns uint16_t Word to program....

#endif // GARMIN_LIDAR-LITE_V3HP_H_INCLUDED
