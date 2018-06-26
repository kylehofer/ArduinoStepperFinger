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

#define MIDDLE_STEP_PIN _BV(0) 			//SCL Pin 3
#define MIDDLE_DIRECTION_PIN _BV(4)		//Pin 4S
#define PROXIMAL_STEP_PIN _BV(1)		//DA Pin 2
#define PROXIMAL_DIRECTION_PIN _BV(7)	//Pin 12

//Microstep Pins
#define MS1_PIN 16
#define MS2_PIN 14
#define MS3_PIN 15

#define BASE_COMPARE_REGISTER 512	//Fastest Step Speed 16th
//#define BASE_COMPARE_REGISTER 256	//Slowest Step Speed 1

#define MOTOR_FLAG 1
#define COMMAND_FLAG _BV(7)

#define STEP_FLAG_BIT _BV(15)
#define DIRECTION_FLAG_BIT _BV(14)
#define FEED_FLAG_BITS 0x3FFF		//Last 13 bits

uint16_t _middleCommands[255] ;
uint16_t _proximalCommands[255];

uint8_t _middleIndex, _proximalIndex, _middleTop, _proximalTop = 0;

bool _isFull;


void setup() 
{
	_isFull = false;

	pinMode(0, INPUT);           // set pin to input
	digitalWrite(0, HIGH);       // turn on pullup resistors

	DDRD |= MIDDLE_STEP_PIN | PROXIMAL_STEP_PIN | MIDDLE_DIRECTION_PIN | PROXIMAL_DIRECTION_PIN; //Setting pins at outputs;

	pinMode(MS1_PIN, OUTPUT);
	pinMode(MS2_PIN, OUTPUT);
	pinMode(MS3_PIN, OUTPUT);

	digitalWrite(MS1_PIN, HIGH);
	digitalWrite(MS2_PIN, HIGH);
	digitalWrite(MS3_PIN, HIGH);

	Serial.begin(115200);

	timerSetup();
}

/*
 *			STEP FUNCTIONS
 */

bool middleStep() {
	if ((_middleCommands[_middleIndex] & FEED_FLAG_BITS) == 0) {
		if ((_middleCommands[_middleIndex] & STEP_FLAG_BIT) > 0)
			PORTD |= MIDDLE_STEP_PIN;

		if ((_middleTop + 1) == _middleIndex)
				_isFull == false;

		_middleIndex++;
		if (_middleIndex == _middleTop)
			stopMiddle();
		else
			OCR1A += _middleCommands[_middleIndex];
	}
	else {
		PORTD &= ~MIDDLE_STEP_PIN;

		if ((_middleCommands[_proximalIndex] & DIRECTION_FLAG_BIT) > 0)
				PORTD |= MIDDLE_DIRECTION_PIN;
		else
				PORTD &= ~MIDDLE_DIRECTION_PIN;

		OCR1A += _middleCommands[_middleIndex];
		_middleCommands[_middleIndex] &= ~FEED_FLAG_BITS;
	}
}

bool proximalStep() {
	if ((_proximalCommands[_proximalIndex] & FEED_FLAG_BITS) == 0) {
		if ((_proximalCommands[_proximalIndex] & STEP_FLAG_BIT) > 0)
			PORTD |= PROXIMAL_STEP_PIN;

		if ((_proximalTop + 1) == _proximalIndex)
				_isFull == false;

		_proximalIndex++;
		if (_proximalIndex == _proximalTop)
			stopProximal();
		else
			OCR1B += _proximalCommands[_proximalIndex];
	}
	else {
		PORTD &= ~PROXIMAL_STEP_PIN;

		if ((_proximalCommands[_proximalIndex] & DIRECTION_FLAG_BIT) > 0)
				PORTD |= PROXIMAL_DIRECTION_PIN;
		else
				PORTD &= ~PROXIMAL_DIRECTION_PIN;

		OCR1B += _proximalCommands[_proximalIndex];
		_proximalCommands[_proximalIndex] &= ~FEED_FLAG_BITS;
	}
}

/*
 *			TIMER FUNCTIONS
 */

ISR(TIMER1_COMPA_vect) {
	middleStep();
}

ISR(TIMER1_COMPB_vect) {
	proximalStep();
}

void timerSetup() {
	noInterrupts();						//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	//TCCR1B = _BV(WGM12) | _BV(CS12);	//CTC Mode with a 256 prescaler	
	TCCR1B = _BV(CS11);	//Normal Mode with a 256 prescaler	
	interrupts();						//Enable interrupts 	
}

void stopMiddle() {
	TIMSK1 &= ~_BV(OCIE1A);				//Output compare register B Disable
}

void stopProximal() {
	TIMSK1 &= ~_BV(OCIE1B);				//Output compare register A Disable
}

void startMiddle() {
	TIFR1 |= _BV(OCF1A);
	TIMSK1 |= _BV(OCIE1A);
	OCR1A = TCNT1 + _middleCommands[_middleIndex];
}

void startProximal() {
	TIFR1 |= _BV(OCF1B);
	TIMSK1 |= _BV(OCIE1B);
	OCR1B = TCNT1 + _proximalCommands[_proximalIndex];
}

/*
 *			COMMUNICATION FUNCTION
 */

void recieveCommand(uint8_t flags, uint16_t data) {

	if ((flags & COMMAND_FLAG) > 0) {
		if ((flags & MOTOR_FLAG) > 0) {
			_middleCommands[_middleTop] = data;
			if (_middleTop == _middleIndex)
				startMiddle();
			_middleTop++;
			if ((_middleTop + 1) == _middleIndex)
				_isFull == true;
		} else {
			_proximalCommands[_proximalTop] = data;
			if (_proximalTop == _proximalIndex)
				startProximal();
			_proximalTop++;
			if ((_proximalTop + 1) == _proximalIndex)
				_isFull == true;
		}
	}	
}

void serialRecieve() {
	while (!_isFull && Serial.available() > 0) {
		uint8_t data = Serial.read();
		recieveCommand(data, (Serial.read() << 8 | Serial.read()));
	}
}

void loop() {
	if (!_isFull && Serial.available() > 0)
		serialRecieve();
}
