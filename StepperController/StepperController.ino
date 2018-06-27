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

#include <inttypes.h>
/*
 * I use the same index for the step and direction in different Port Data registers
 * while using the same position for the data flags to slightly increase the interrupt functions. 
 */
#define STEP_PIN _BV(1)			//PB1 for Middle, PD1 for Proximal
#define DIRECTION_PIN _BV(3)	//PB3 for Middle, PD3 for Proximal
#define CLEAR_STEP_DIRECTION_PIN 0b11110101

//Microstep Pins
#define MS1_PIN 5
#define MS2_PIN 4
#define MS3_PIN 3

#define BASE_COMPARE_REGISTER 512	//Fastest Step Speed 16th

#define MOTOR_FLAG_BIT 1
#define STEP_FLAG_BIT _BV(1)
#define DIRECTION_FLAG_BIT _BV(3)
//#define COMPLETE_FLAG_BIT _BV(3)
#define BUFFER_CHECK_FLAG _BV(6)
#define COMMAND_FLAG _BV(7)
#define SRAM_LIMIT 80				//Limit to avoid overflowing the SRAM
#define FEED_FLAG_BITS 0x3FFF		//Last 13 bits

/*
 * Struct used to hold the commands for stepper instructions.
 * Commands are held in two seperate linked queues.
 * Commands are made up of
 */
struct Command {
public:
	uint16_t feed;
	uint8_t flags;
	Command *tail;

	Command(uint8_t flags, uint16_t feed) {
		this->feed = feed;
		this->flags = flags;
		this->tail = NULL;
	}
} *_middleHead = NULL, *_middleTail = NULL
, *_proximalHead = NULL, *_proximalTail = NULL;

uint8_t _commandCount;
bool _ready;

void setup() {

	_commandCount = 0;
	_ready = true;

	DDRB |= (STEP_PIN | DIRECTION_PIN);	//Setting Middle pins at outputs
	DDRD |= (STEP_PIN | DIRECTION_PIN);	//Setting Proximal pins at outputs

	//Setting the microstepper values for my stepper driver
	pinMode(MS1_PIN, OUTPUT);
	pinMode(MS2_PIN, OUTPUT);
	pinMode(MS3_PIN, OUTPUT);

	digitalWrite(MS1_PIN, HIGH);
	digitalWrite(MS2_PIN, HIGH);
	digitalWrite(MS3_PIN, HIGH);

	//Serial.begin(115200);
	Serial.begin(250000);

	timerSetup();
}

/*
 *			TIMER FUNCTIONS
 */

//Old function to congregate the two timers into a single easier to edit function
//However the extra overhead added by this function severely effected performance.

bool step(Command **head, uint8_t *pinRegister, uint16_t *compareRegister) {
	if ((*head)->feed == 0) {
		*pinRegister |= (*head)->flags & STEP_FLAG_BIT;

		Command *tail = (*head)->tail;
		delete (*head);
		(*head) = NULL;

		_commandCount--;

		if (tail) {
			(*head) = tail;
			*compareRegister += (*head)->feed;
		}
		else
			return false;
	}
	else {
		*pinRegister &= CLEAR_STEP_DIRECTION_PIN;
		*pinRegister |= ((*head)->flags & DIRECTION_FLAG_BIT);
		*compareRegister += (*head)->feed;
		(*head)->feed = 0;
	}
	return true;
}

/* 
 *						Timer1 compare A
 * The steps are handled in two phases to create an accurate PWM signal.
 * The first phase involves bringing the signal low, and setting the direction
 * The second phase involves bringing the signal high, setting the next command
 */
ISR(TIMER1_COMPA_vect) {
	if (_middleHead->feed > 0) { //Phase 1
		PORTB &= CLEAR_STEP_DIRECTION_PIN;				//Clear Step and Direction
		PORTB |= (_middleHead->flags & DIRECTION_FLAG_BIT);	//Set Direction
		OCR1A += _middleHead->feed;
		_middleHead->feed = 0;
	}
	else { //Phase 2

		PORTB |= _middleHead->flags & STEP_FLAG_BIT;	//Raising step if it's enabled in the flags

		Command *tail = _middleHead->tail;

		delete _middleHead;								//Freeing Sram
		_middleHead = NULL;

		_commandCount--;

		if (tail) {										//Proceeding to the next command
			_middleHead = tail;
			OCR1A += _middleHead->feed;
		}
		else {											//End of commands
			_middleTail = NULL;
			stopMiddle();
		}
	}
}

//Timer1 compare B
ISR(TIMER1_COMPB_vect) {
	if (_proximalHead->feed == 0) {
		PORTD |= (_proximalHead->flags & STEP_FLAG_BIT);
		

		Command *tail = _proximalHead->tail;

		delete _proximalHead;
		_proximalHead = NULL;

		_commandCount--;

		if (tail) {			
			_proximalHead = tail;
			OCR1B += _proximalHead->feed;
		}
		else {
			_proximalTail = NULL;
			stopProximal();
		}
	}
	else {		
		PORTD &= CLEAR_STEP_DIRECTION_PIN;
		PORTD |= (_proximalHead->flags & DIRECTION_FLAG_BIT);
		OCR1B += _proximalHead->feed;
		_proximalHead->feed = 0;
	}
}

void timerSetup() {
	noInterrupts();						//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TIFR1 |= _BV(OCF1A) | _BV(OCF1B);	//Clearing flagged interrupts	
	TCCR1B = _BV(CS11);					//Normal Mode with a 256 prescaler
	OCR1A = 0xFFFF;
	OCR1B = 0xFFFF;
	interrupts();						//Enable interrupts 	
}

void stopMiddle() {
	TIMSK1 &= ~_BV(OCIE1A);				//Output compare register B Disable
}

void stopProximal() {
	TIMSK1 &= ~_BV(OCIE1B);				//Output compare register A Disable
}

void startMiddle(uint16_t feed) {
	TIMSK1 |= _BV(OCIE1A);				//Output compare register A Enable
	OCR1A = TCNT1 + feed;				//Start timer for first step
}

void startProximal(uint16_t feed) {
	TIMSK1 |= _BV(OCIE1B);				//Output compare register B Enable
	OCR1B = TCNT1 + feed;				//Start timer for first step
}

/*
 *			COMMUNICATION FUNCTIONS
 */

//Adds commands received from the serial connection onto the command queue
void addToQueue(uint8_t flags, uint16_t feed) {
	Command *insert = new Command(flags, feed);
	if ((flags & MOTOR_FLAG_BIT) == 0) {	//Middle Command
		if (_middleTail) {					//Tail exists, add command onto the tail
			_middleTail->tail = insert;
			_middleTail = _middleTail->tail;
		}
		else if (_middleHead) {				//Head exists, add command onto the head
			_middleHead->tail = insert;
			_middleTail = insert;
		}
		else {								//Head doesn't exists, add command as the head
			_middleHead = insert;			
			startMiddle(feed);				//Start timer
		}
	} else {								//Proximal Command
		if (_proximalTail) {
			_proximalTail->tail = insert;
			_proximalTail = _proximalTail->tail;
		}
		else if (_proximalHead) {
			_proximalHead->tail = insert;
			_proximalTail = insert;
		}
		else {
			_proximalHead = insert;
			startProximal(feed);
		}
	}
	_commandCount++;
}

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

void readSerial() {	
	uint8_t length = Serial.available();
	if (length > 0) {
		byte input[length];
		Serial.readBytes(input, length);				//Reading all data available

		if ((length + _commandCount) < SRAM_LIMIT)	//Estimating SRAM Usage
			Serial.write(1);
		else
			_ready = false;

		int i = 0;										//Loops through inputted data
		while(i < length) {
			uint8_t flags = input[i++];
			if ((flags & COMMAND_FLAG) > 0) {
				uint16_t feed = (input[i++] << 8);
				feed |= input[i++];
				addToQueue(flags, feed);
			}
		}
	}
}

void loop() {
	if (!_ready) {
		if (_commandCount < SRAM_LIMIT) {				//SRAM clear, resume communication
			_ready = true;
			Serial.write(1);
		}
	}
	else
		readSerial();
}
