/*---------------------------------------------------------------------------------------------------------
- Garmin LIDAR-LITE v.3HP Drivers,                                                                        -
- Operating temperature: -20° to 60°C (-4 to 140°F)                                                       -
- Power:5 Vdc nominal, 4.5 Vdc min., 5.5 Vdc max.                                                         -
- Current consumption: 65mA idle, 85mA during an acquisition                                              -
- Range: (70% reflective target) - 40 m (131 ft.)                                                         -
- Resolution: ±1 cm (0.4 in.)                                                                             -
- Accuracy: < 2m ±5cm (2 in.) typical, NOTE: Nonlinearity present below 1 m (39.4 in.)                    -
- --------- Laser Specifications ------------------------------------------------------------------------ -
- Wavelength: 905 nm (nominal)                                                                            -
- Total laser power (peak): 1.3 W                                                                         -
- Pulse width: 0.5 us (50% duty cycle)                                                                    -
- Pulse train repetition frequency: 10-20 kHz nominal                                                     -
- Energy per pulse: <280 nJ                                                                               -
- Beam diameter at laser aperture: 12 × 2 mm (0.47 × 0.08 in.)                                            -
- Divergence: 8 mRad                                                                                      -
- -------- Interface Specifications --------------------------------------------------------------------- -
- i2C interface: 400 kHz frequency                                                                        -
- PWM interface: available                                                                                -
- ------------------------------------------------------------------------------------------------------- -
- written by Dr.-Ing. David Zohrabyan, Potsdam 17.02.2021 at Physalis Labs.                               -
- ------------------------------------------------------------------------------------------------------- -
- start by doing what is necessary; then do what is possible; and suddenly,                               -
- you are doing the impossible.                                                                           -
---------------------------------------------------------------------------------------------------------*/
#include <avr/io.h>
#include <util/twi.h>
#include <USART.h>
#include <Garmin_LIDAR-Lite_v3HP.h>
// ----------------------------------- setup LIDAR to start measuring ---------------------------------- //
void LIDAR_scanArea(void){

    uint8_t receiver_reply;

    i2cStart();                                                          // start a session on the i2C BUS
    i2cSend(0b11000100);
                                                                               // LIDAR address with write
    receiver_reply = TWSR;                                                         // read twi status code
    receiver_reply = (receiver_reply & 0b11111000);                               // mask the lower 3 bits

    if(receiver_reply == TW_MT_SLA_ACK){                        // if LIDAR sent Ack! proceed to next step
            printString("issuing address + write bit >> ok!, device accessible\r\n");
            printBinaryByte(receiver_reply);                       // return the upper 5 bits for check-up
            printString("\r\n");

            i2cSend(0b00000000);                                   // send 0x00 register address, on LIDAR

            receiver_reply = TWSR;                                                 // read twi status code
            receiver_reply = receiver_reply & 0b11111000;                         // mask the lower 3 bits

            if(receiver_reply == TW_MT_DATA_ACK){               // if LIDAR sent Ack! proceed to next step
                    printString("register 0x00 address sent >> ok!\r\n");
                    printBinaryByte(receiver_reply);               // return the upper 5 bits for check-up
                    printString("\r\n");

                    i2cSend(0b00000100);  // writing 0x04 to register 0x00 to start a distance measurement

                    receiver_reply = TWSR;
                    receiver_reply = receiver_reply & 0b11111000;

                    if(receiver_reply == TW_MT_DATA_ACK){       // if LIDAR sent Ack! proceed to next step
                            printString("wrote 0x04 to register 0x00\r\n");
                            printBinaryByte(receiver_reply);
                            printString("\r\n");
                            i2cStop();                                      // stop session on the i2C BUS
                        }
                }
              }
}
//----------------------------------- reading LIDAR status register -------------------------------------//
uint8_t check_LIDAR_busyFlag(void){

    uint8_t receiver_reply;
    uint8_t status_register = 0;

    i2cStart();                                                          // start a session on the i2C BUS
    i2cSend(0b11000100);                                                       // write operation on LIDAR

    receiver_reply = TWSR;                                                         // read twi status code
    receiver_reply = (receiver_reply & 0b11111000);                               // mask the lower 3 bits

    if(receiver_reply == TW_MT_SLA_ACK){                        // if LIDAR sent Ack! proceed to next step
            printString("issuing address + write bit >> ok!, device accessible\r\n");
            printBinaryByte(receiver_reply);                       // return the upper 5 bits for check-up
            printString("\r\n");

            i2cSend(0b00000001);                             // sending LIDAR status register address here

            receiver_reply = TWSR;                                                 // read twi status code
            receiver_reply = receiver_reply & 0b11111000;                         // mask the lower 3 bits

            if(receiver_reply == TW_MT_DATA_ACK){               // if LIDAR sent Ack! proceed to next step
                    printString("register 0x01 address sent >> ok!\r\n");
                    printBinaryByte(receiver_reply);               // return the upper 5 bits for check-up
                    printString("\r\n");
                    i2cStop();                                                         // stop i2C session
                }
            }
            printString("i am here");
            printString("\r\n");

            i2cStart();                                                       // start a i2C session again
            i2cSend(0b11000101);                                        // send LIDAR address and read bit

            receiver_reply = TWSR;                                                 // read twi status code
            receiver_reply = receiver_reply & 0b11111000;                         // mask the lower 3 bits

            printBinaryByte(receiver_reply);                       // return the upper 5 bits for check-up
            printString("\r\n");

            if(receiver_reply == TW_MR_SLA_ACK){                // if LIDAR sent Ack! proceed to next step
                    printString("issuing address + read bit >> ok!, device accessible ------\r\n");
                    printBinaryByte(receiver_reply);               // return the upper 5 bits for check-up
                    printString("\r\n");
                    status_register = i2cReadNoAck();
                    printString("\r\nLIDAR-STATUS = ");
                    printBinaryByte(status_register);
                    printString("\r\n");
            }
            return(status_register);
}
//--------------------------------- reading LIDAR status register ends here ----------------------------//
//--------------------------------- distance measurement read-cycle begins here ------------------------//
uint16_t LIDAR_shuttle_Data(void){

    uint8_t receiver_reply;
    uint8_t lidar_highbyte;
    uint8_t lidar_lowbyte;
    uint16_t distance = 0;

    i2cStart();                                                          // start a session on the i2C BUS
    i2cSend(0b11000100);                                                       // LIDAR address with write

    receiver_reply = TWSR;                                                         // read twi status code
    receiver_reply = (receiver_reply & 0b11111000);                               // mask the lower 3 bits

    if(receiver_reply == TW_MT_SLA_ACK){                        // if LIDAR sent Ack! proceed to next step
            printString("issuing address + write bit >> ok!, device accessible\r\n");
            printBinaryByte(receiver_reply);                       // return the upper 5 bits for check-up
            printString("\r\n");
    }

    i2cSend(0b10001111);                                          // send 0x8F register address...on LIDAR

    receiver_reply = TWSR;                                                         // read twi status code
    receiver_reply = receiver_reply & 0b11111000;                                 // mask the lower 3 bits

    if(receiver_reply == TW_MT_DATA_ACK){                       // if LIDAR sent Ack! proceed to next step
            printString("register 0x8F address sent >> ok!\r\n");
            printBinaryByte(receiver_reply);                       // return the upper 5 bits for check-up
            printString("\r\n");
            i2cStop();
            }

            i2cStart();
            i2cSend(0b11000101);                                   // setup LIDAR into read mode operation

            receiver_reply = TWSR;
            receiver_reply = receiver_reply & 0b11111000;

            if(receiver_reply == TW_MR_SLA_ACK){                // if LIDAR sent Ack! proceed to next step
                    printString("issuing address + read bit >> ok!, device accessible *****\r\n");
                    printBinaryByte(receiver_reply);
                    printString("\r\n");
                    lidar_highbyte = i2cReadAck();
                    lidar_lowbyte = i2cReadNoAck();
                    i2cStop();
                    distance = lidar_highbyte << 8;
                    distance = distance | lidar_lowbyte;
                    printString("distance = ");
                    printWord(distance);
                    printString("\r\n");
                }
                return(distance);
}
