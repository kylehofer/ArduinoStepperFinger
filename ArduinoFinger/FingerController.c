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

#include "FingerController.h"

static Control_Data_t CONTROL_DATA = {
	.next = 1,			//Needs to be different from middle/prox head
	.middleHead = 0,
	.middleTail = 0,
	.proximalHead = 0,
	.proximalTail = 0,
	.middleActive = 0,
	.proximalActive = 0
};

static Command_List_t COMMAND_LIST[COMMAND_COUNT];

void controllerSetup() {
	//Setting the microstepper values for the stepper driver
	DDRB |= (STEP_PIN | DIRECTION_PIN);	//Setting Middle pins at outputs
	DDRD |= (STEP_PIN | DIRECTION_PIN | MS1_PIN | MS2_PIN | MS3_PIN);	//Setting Proximal pins, and microstep pins at outputs

	PORTD |= MS1_PIN | MS2_PIN | MS3_PIN;	//Setting microset pin values

	timerSetup();
}

/*
 *			SIGNAL FUNCTIONS
 */

/*
 * The steps are handled in two phases to create an accurate PWM signal.
 * The first phase involves bringing the signal low, and setting the direction
 * The second phase involves bringing the signal high, setting the next command
 */

ISR(TIMER1_COMPA_vect) {
	if (MIDDLE_FEED > 0) {							//Phase 1
		PORTB = (PORTB & CLEAR_STEP_DIRECTION_PINS)
			| (MIDDLE_FLAGS & DIRECTION_FLAG_BIT);	//Setting Direction, clearing step
		OCR1A += MIDDLE_FEED;						//Incrementing interrupt register
		MIDDLE_FEED = 0;							//Enabling phase 2
	}
	else {											//Phase 2
		PORTB |= (MIDDLE_FLAGS & STEP_FLAG_BIT);	//Setting Step Pin
		if (MIDDLE_HEAD != MIDDLE_TAIL) {			//Checking for additional commands
			MIDDLE_HEAD = MIDDLE_HEAD_NEXT;			//Incrementing Command
			OCR1A += MIDDLE_FEED;					//Incrementing interrupt register
		}
		else {										//Out of Commands
			MIDDLE_ACTIVE = 0;						//Set flag
			TIMSK1 &= ~_BV(OCIE1A);					//Disable Interrupt
		}
	}	
}


ISR(TIMER1_COMPB_vect) {
	if (PROXIMAL_FEED > 0) {
		PORTD = (PORTD & CLEAR_STEP_DIRECTION_PINS)
			| (PROXIMAL_FLAGS & DIRECTION_FLAG_BIT);
		OCR1B += PROXIMAL_FEED;
		PROXIMAL_FEED = 0;
	}
	else {
		PORTD |= (PROXIMAL_FLAGS & STEP_FLAG_BIT);

		if (PROXIMAL_HEAD != PROXIMAL_TAIL) {
			PROXIMAL_HEAD = PROXIMAL_HEAD_NEXT;
			OCR1B += PROXIMAL_FEED;
		}
		else {
			PROXIMAL_ACTIVE = 0;
			TIMSK1 &= ~_BV(OCIE1B);
		}
	}	
}

/*
 *			TIMER FUNCTIONS
 */

void timerSetup() {
	cli();								//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TIFR1 |= _BV(OCF1A) | _BV(OCF1B);	//Clearing flagged interrupts	
	TCCR1B = _BV(CS11);					//Normal Mode with a 256 prescaler
	sei();								//Enable interrupts 	
}

/*
 *			COMMUNICATION FUNCTIONS
 */

//Adds commands received from the serial connection into the command queue
void addToQueue(uint8_t flags, uint16_t feed) {	

	NEXT_COMMAND = (Command_List_t){ flags, feed, 0 };	//Next Command

	if ((flags & MOTOR_FLAG_BIT) == 0) {				//Middle Command
		if (MIDDLE_ACTIVE)
			MIDDLE_TAIL_NEXT = CONTROL_DATA.next;		//Add to the end of the queue
		else {
			MIDDLE_HEAD = CONTROL_DATA.next;			//Start the queue
			MIDDLE_ACTIVE = 1;
			TIMSK1 |= _BV(OCIE1A);						//Output compare register A Enable
			OCR1A = TCNT1 + feed;						//Set interrupt register for first step
		}
		MIDDLE_TAIL = CONTROL_DATA.next;						

	} else {											//Proximal Command
		if (PROXIMAL_ACTIVE)
			PROXIMAL_TAIL_NEXT = CONTROL_DATA.next;
		else {
			PROXIMAL_HEAD = CONTROL_DATA.next;
			PROXIMAL_ACTIVE = 1;
			TIMSK1 |= _BV(OCIE1B);
			OCR1B = TCNT1 + feed;
		}
		PROXIMAL_TAIL = CONTROL_DATA.next;		
	}
	CONTROL_DATA.next++;	
}


uint8_t isFingerReady() {
	return (CONTROL_DATA.next != MIDDLE_HEAD || CONTROL_DATA.next != PROXIMAL_HEAD);
}