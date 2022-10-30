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

void	reportUART_12bytes (uint8_t telemetry_type);
