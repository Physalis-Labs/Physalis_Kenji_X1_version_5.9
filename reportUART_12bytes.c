/***************************************************************************************
** Function name:           reportUART_12bytes
** Description:             AVR->ESP32 telemetry assembler, sends 12 bytes UART frames
** 							[#]		[telemetry_type]	[telemetry_value]	[timestamp]	[reserved]	[$]
**	96 bits/12 bytes		[8 bit] [8 bit]				[32 bit]			[32 bit]	[8bit]		[8bit]
**
* telemetry_type:			[dec]49 [char]1 	Battery
**							[dec]50 [char]2		Hall sensors
**							[dec]51 [char]3		Ambient light sensors
**							[dec]52 [char]4		IR distance sensor
***************************************************************************************/
#include <avr/io.h>
#include <USART.h>
#include <battery_pack.h>
#include <Hardware_Bay.h>

extern volatile short hall_S2R_pulse;                               // variables for counting Hall sensor pulses /right
extern volatile short hall_S2L_pulse;                                // variables for counting Hall sensor pulses /left
extern uint16_t ired_range;                                     // analog data from IRED sensor (sharp)
extern uint16_t ambient_light;                                                                 // photo-conductive cell
extern const uint16_t notes[];
extern uint16_t currentNoteLength;
extern void playNote(uint16_t period, uint16_t duration);

void	reportUART_12bytes (uint8_t telemetry_type)
{
  union				Shared32bits							// union declaration for shared access to the same 32 bits
		{
		float		shared32bit;							// float for storing 32 bit representation of telemetry_value
		uint8_t	shared8bit[4];								// array of 4 bytes representation of telemetry_value
		uint16_t shared16bit[2];							// array of 2 words representation of telemetry_value
		};
  union Shared32bits	telemetry;							// declare "telemetry" as an instance of union "Shared32bits"

  struct	Timestamp_short									// Timestamp_short structure declaration, storing 4 lower bytes of RTC
		{
		uint8_t		rtchsec;								// RTCHSEC, 	00h		HUNDREDTH OF SECONDS
		uint8_t		rtcsec;									// RTCSEC, 		01h		SECONDS
		uint8_t		rtcmin;									// RTCMIN, 		02h		MINUTES
		uint8_t		rtchour;								// RTCHOUR, 	03h		HOURS
		};
  struct 	Timestamp_short		telemetry_timestamp;		// define telemetry_timestamp as an instance of Timestamp_short structure

 //		alternate float to  byte trimming option example
 // 	transmitByte( (uint8_t) (telemetry_value >> 24) );       // MSB most significant byte
    switch(telemetry_type){
                case 	49:											// telemetry - battery - "1"
																	// telemetry UART constructor for battery
							telemetry.shared32bit = oversample16x() * VOLTAGE_DIV_FACTOR * REF_VCC / 4096;          // power supply voltage
                        break;
                case 	50:											// telemetry - hall sensors - "2"
																	// telemetry UART constructor for hall sensors
							telemetry.shared16bit[1] = hall_S2R_pulse;												// Hall Sensor - Right pulses
							telemetry.shared16bit[0] = hall_S2L_pulse;												// Hall Sensor - Left pulses
                        break;
                case 	51:											// telemetry - ambient light sensors - "3"
																	// telemetry UART constructor for ambient light sensors
							telemetry.shared16bit[0] = fetchDataADC(AMBIENT_LIGHT);									// Ambient Light Sensor
							telemetry.shared16bit[0] = ired_range;	// lower word holds ambient light value, variables swapped
                        break;
                case 	52:					// telemetry - IR distance sensor - "4"
											// telemetry UART constructor for IR distance sensor
							telemetry.shared16bit[0] = fetchDataADC(IRED_SENSOR);										// IR distance sensor
							telemetry.shared16bit[0] = ambient_light;	// lower word holds IR distance value, variables swapped
                        break;
                default  :					// unrecognized telemetry type
						playNote(notes[9], currentNoteLength);
						playNote(notes[9], currentNoteLength);
						playNote(notes[9], currentNoteLength);
        }
  telemetry_timestamp.rtchsec = 0;				// RTCHSEC placeholder
  telemetry_timestamp.rtcsec = 0;				// RTCSEC placeholder
  telemetry_timestamp.rtcmin = 0;				// RTCMIN placeholder
  telemetry_timestamp.rtchour = 0;				// RTCHOUR placeholder

  transmitByte('#');							// UART Telemetry message start					#
  transmitByte(telemetry_type);					// transmit telemetry type						1-4

  transmitByte(telemetry.shared8bit[3]);		// transmit telemetry high byte 	[X][][][]
  transmitByte(telemetry.shared8bit[2]);		// transmit telemetry second byte [][X][][]
  transmitByte(telemetry.shared8bit[1]);		// transmit telemetry third byte 	[][][X][]
  transmitByte(telemetry.shared8bit[0]);		// transmit telemetry low byte 		[][][][X]

  transmitByte(telemetry_timestamp.rtchsec);	// transmit timestamp RTCHSEC byte
  transmitByte(telemetry_timestamp.rtcsec);		// transmit timestamp RTCSEC byte
  transmitByte(telemetry_timestamp.rtcmin);		// transmit timestamp RTCMIN byte
  transmitByte(telemetry_timestamp.rtchour);	// transmit timestamp RTCHOUR byte

  transmitByte(0);								// reserved byte for 12 bytes messages	 0
  transmitByte(36);								// UART Telemetry message end					   $
}
