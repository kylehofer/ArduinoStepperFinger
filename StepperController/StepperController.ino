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

#define M1_STEP_PIN _BV(0) 		//SCL Pin 3
#define M1_DIRECTION_PIN _BV(4)	//Pin 4S
#define M2_STEP_PIN _BV(1)		//DA Pin 2
#define M2_DIRECTION_PIN _BV(7)	//Pin 12

//Microstep Pins
#define MS1_PIN 16
#define MS2_PIN 14
#define MS3_PIN 15

#define NUM_MOTORS 2
#define COMMAND_BYTE_SIZE 6
#define DBL_COMMAND_BYTE_SIZE (COMMAND_BYTE_SIZE << 1)
#define MEMORY_LIMIT 500

#define TIME_OUT 500

#define BASE_COMPARE_REGISTER 16	//Fastest Step Speed 16th
//#define BASE_COMPARE_REGISTER 256	//Slowest Step Speed 1

#define STEP_FLAG_BIT _BV(5)		
#define DIRECTION_FLAG_BIT _BV(4)	
#define FEED_FLAG_BITS 0b1111
#define DATA_TYPE_FLAG _BV(7)
#define MOTOR_FLAG _BV(6)
#define START_OF_MESSAGE 1
#define END_OF_MESSAGE _BV(1)

class Stepper {

private:
	class Command {
		Command *_next;
		uint8_t _count;
		uint8_t _flags;

	public:
		Command(uint8_t flags, uint8_t count) {
			_count = count;
			_flags = flags;
			_next = NULL;
		}

		Command* next() {
			return _next;
		}

		void add(class Command *command) {
			if (!_next)
				_next = command;
			else
				_next->add(command);
		}

		uint8_t step() {
			_count--;
		}

		uint8_t getCount() {
			return _count;
		}

		uint8_t getFeed() {
			return (_flags & FEED_FLAG_BITS);
		}

		bool getDirection() {
			return (_flags & DIRECTION_FLAG_BIT);
		}

		bool getStep() {
			return (_flags & STEP_FLAG_BIT);
		}
	};

	class Command *_head;
	int8_t _directionPin;
	int8_t _stepPin;
	bool _isStep;

	void updateDirection(Command *command) {
		if (command->getDirection())
				PORTD |= _directionPin;
		else
				PORTD &= ~_directionPin;
	}

public:

	Stepper(int8_t stepPin, int8_t directionPin) {
		_directionPin = directionPin;
		_stepPin = stepPin;
		_head = NULL;
		_isStep = true;
	}

	void addCommand(uint8_t flags, uint8_t count) {
		class Command *command = new Command(flags, count);
		if (!_head) {
			_head = command;
			updateDirection(_head);
		}
		else
			_head->add(command);
	}

	bool step() {
			if (!_isStep) {
				PORTD &= ~_stepPin;
				_isStep = true;
				if (_head->getCount() == 0)
					return false;
			} 
			else {
				if (_head->getStep()) PORTD |= _stepPin;
				_isStep = false;
				_head->step();
			}
		return true;
	}

	bool next() {
		Command *head = _head;
		if (head) {
			_head = head->next();
			delete head;
			if (_head) {
				updateDirection(_head);
				return true;
			}
		}
		return false;
	}

	uint8_t getFeed() {
		return (_head) ? _head->getFeed() : 1;
	}

	uint8_t getCount() {
		return (_head) ? _head->getCount() : 0;
	}
};

uint8_t _timer;
uint16_t _timeOutCount;

uint16_t _feedRate;
uint16_t _OCR1BMax;

bool _isTimer;
bool _isTransmission;
bool _dataRequested;

class Stepper *_middle, *_proximal;
bool _hasMiddle, _hasProximal;

bool (*timer1COMPA)();
bool (*timer1COMPB)();

uint16_t _commandMemory;


void setup() 
{
	_timer = 0;
	_isTransmission = false;
	_dataRequested = false;

	_feedRate = BASE_COMPARE_REGISTER;

	pinMode(0, INPUT);           // set pin to input
	digitalWrite(0, HIGH);       // turn on pullup resistors

	DDRD |= M1_STEP_PIN | M2_STEP_PIN | M1_DIRECTION_PIN | M2_DIRECTION_PIN; //Setting pins at outputs;

	pinMode(MS1_PIN, OUTPUT);
	pinMode(MS2_PIN, OUTPUT);
	pinMode(MS3_PIN, OUTPUT);

	digitalWrite(MS1_PIN, HIGH);
	digitalWrite(MS2_PIN, HIGH);
	digitalWrite(MS3_PIN, HIGH);

	Serial.begin(115200);

	_middle = new Stepper(M1_STEP_PIN, M1_DIRECTION_PIN);
	_proximal = new Stepper(M2_STEP_PIN, M2_DIRECTION_PIN);

	timerSetup();
}

/*
 *			STEP FUNCTIONS
 */

void updateFeed() {
	uint8_t mFeed = _middle->getFeed(), pFeed = _proximal->getFeed();

	_hasMiddle = true;//(_middle->getCount() > 0);
	_hasProximal = true;//(_proximal->getCount() > 0);

	if (mFeed < pFeed) {
		//_OCR1BMax = _feedRate;

		OCR1A = (_feedRate * pFeed) / mFeed;		
		timer1COMPB = &middleStep;
		timer1COMPA = &proximalStep;
	} else {
		OCR1A = (_feedRate  * mFeed) / pFeed;			
		timer1COMPB = &proximalStep;
		timer1COMPA = &middleStep;
	}
}

bool nextCommand() {
	_commandMemory -= DBL_COMMAND_BYTE_SIZE;
	if (_middle->next() & _proximal->next()) {
		updateFeed();
		return true;
	}
	return false;
}

bool middleStep() {
	if (!_hasMiddle || !_middle->step()) {
		if (!_hasProximal) {
			return nextCommand();
		}
		else
			_hasMiddle = false;
	}
	return true;
}

bool proximalStep() {
	
	if (!_hasProximal || !_proximal->step()) {
		if (!_hasMiddle) {
			return nextCommand();
		}
		else
			_hasProximal = false;
	}
	return true;
}

/*
 *			TIMER FUNCTIONS
 */

ISR(TIMER1_COMPB_vect) {
	OCR1B += _OCR1BMax;
	if (!(*timer1COMPB)())
		timerStop();
}

ISR(TIMER1_COMPA_vect) {
	OCR1B -= OCR1A;
	if (!(*timer1COMPA)())
		timerStop();
}

void timerSetup() {
	noInterrupts();						//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1B = _BV(WGM12) | _BV(CS12);	//CTC Mode with a 256 prescaler	

	_OCR1BMax = _feedRate;
	OCR1B = _feedRate;

	//
	interrupts();						//Enable interrupts 	
}

void timerStart() {
	_isTimer = true;
	noInterrupts();						//Disable interrupts
	updateFeed();
	TCNT1 = 0;

	TIFR1 |= _BV(OCF1A) | _BV(OCF1B);	//Clearing interrupt flags

	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);//Output compare register A enabled

	interrupts();						//Enable interrupts
}

void timerStop() {
	_isTimer = false;
	noInterrupts();						//Disable interrupts
	TIMSK1 &= ~_BV(OCIE1A);				//Output compare register A Disable
	TIMSK1 &= ~_BV(OCIE1B);				//Output compare register A Disable
	interrupts();						//Enable interrupts
}

/*
 *			COMMUNICATION FUNCTION
 */

void recieveCommands(uint8_t flags, uint8_t count) {
	if ((flags & MOTOR_FLAG) > 0) {
		_middle->addCommand(flags, count);
	} else {
		_proximal->addCommand(flags, count);
	}
}

void serialRecieve() {

	bool commandRecieved = false;

	while (_commandMemory < MEMORY_LIMIT && Serial.available() > 0) {
		uint8_t data = Serial.read();
		if ((data & DATA_TYPE_FLAG) > 0) {
			recieveCommands(data, Serial.read());
			_commandMemory += COMMAND_BYTE_SIZE;
			commandRecieved = true;
		}
	}
	if (commandRecieved & !_isTimer)
		timerStart();
}

void loop() {
	delay(1);
	if (Serial.available() > 0) {
		serialRecieve();
	}
}
