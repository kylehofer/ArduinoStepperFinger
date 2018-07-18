/*
 * Copyright (c) 2018 Kyle Hofer
 * Email: kylehofer@neurak.com.au
 * Web: https://neurak.com.au/
 *
 * This file is part of ArduinoStepperFinger.
 *
 * ArduinoStepperFinger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArduinoStepperFinger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * I use the same index for the step and direction in different Port Data registers
 * while using the same position for the data flags to slightly increase the interrupt functions. 
 */

/*
 * I ran into a lot of issues trying to nail down a
 * reliable communication protocol.
 * Keeping the arduino's serial communications up with 
 * the speed required for 1/16th microsteps was a real pain.
 * Alongside this, avoiding buffer overflows, and running
 * out of SRAM.
 * Using a handshaking approach, the Arduino will respond
 * after recieving data letting the host know it is ready to 
 * recieve more data. The data is recieved in blocks
 * Pauses transmissions if it'll get close to filling the SRAM.
 */

#ifndef FINGERCONTROLLER_H_
#define FINGERCONTROLLER_H_

#include <avr/interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * I use the same index for the step and direction in different Port Data registers
 * while using the same position for the data flags to slightly increase the interrupt functions. 
 */
#define STEP_PIN _BV(1)			//PB1 for Middle, PD1 for Proximal
#define DIRECTION_PIN _BV(3)	//PB3 for Middle, PD3 for Proximal
#define CLEAR_STEP_DIRECTION_PINS 0b11110101

//Microstep Pins
#define MS1_PIN 0				//PD0 Pin 3
#define MS2_PIN _BV(4)			//PD4 Pin 4
#define MS3_PIN _BV(7)			//PD7 Pin 6

#define BASE_COMPARE_REGISTER 512	//Fastest Step Speed 16th

#define MOTOR_FLAG_BIT 1
#define STEP_FLAG_BIT _BV(1)
#define DIRECTION_FLAG_BIT _BV(3)
#define BUFFER_CHECK_FLAG _BV(6)
#define FEED_FLAG_BITS 0x3FFF		//Last 13 bits

//Macros to access the static Control data members to improve readability
#define NEXT_COMMAND COMMAND_LIST[CONTROL_DATA.next]

#define MIDDLE_ACTIVE CONTROL_DATA.middleActive
#define MIDDLE_HEAD CONTROL_DATA.middleHead
#define MIDDLE_TAIL CONTROL_DATA.middleTail
#define MIDDLE_HEAD_NEXT COMMAND_LIST[CONTROL_DATA.middleHead].next
#define MIDDLE_TAIL_NEXT COMMAND_LIST[CONTROL_DATA.middleTail].next
#define MIDDLE_FEED COMMAND_LIST[CONTROL_DATA.middleHead].feed
#define MIDDLE_FLAGS COMMAND_LIST[CONTROL_DATA.middleHead].flags

#define PROXIMAL_ACTIVE CONTROL_DATA.proximalActive
#define PROXIMAL_HEAD CONTROL_DATA.proximalHead
#define PROXIMAL_TAIL CONTROL_DATA.proximalTail
#define PROXIMAL_HEAD_NEXT COMMAND_LIST[CONTROL_DATA.proximalHead].next
#define PROXIMAL_TAIL_NEXT COMMAND_LIST[CONTROL_DATA.proximalTail].next
#define PROXIMAL_FEED COMMAND_LIST[CONTROL_DATA.proximalHead].feed
#define PROXIMAL_FLAGS COMMAND_LIST[CONTROL_DATA.proximalHead].flags

#define COMMAND_COUNT 0x100

typedef struct {
	uint8_t flags;
	uint16_t feed;	
	uint8_t next;
} Command_List_t;

typedef struct {
	uint8_t next, middleHead, middleTail, proximalHead, proximalTail;
	volatile uint8_t middleActive, proximalActive;
} Control_Data_t;

void controllerSetup();
void timerSetup();
void startMiddle(uint16_t feed);
void startProximal(uint16_t feed);
void addToQueue(uint8_t flags, uint16_t feed);
uint8_t isFingerReady();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* FINGERCONTROLLER_H_ */