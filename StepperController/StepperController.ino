#include <inttypes.h>
#include <Wire.h>

#define M1_STEP_PIN 9
#define M1_DIRECTION_PIN 8
#define M2_STEP_PIN 7
#define M2_DIRECTION_PIN 6

//Microstep Pins
#define MS1_PIN 16
#define MS2_PIN 14
#define MS3_PIN 15

#define NUM_MOTORS 2

#define TIME_OUT 500

#define BASE_COMPARE_REGISTER 18	//Fastest Step Speed

#define STEP_FLAG_BIT _BV(5)		
#define DIRECTION_FLAG_BIT _BV(4)	
#define FEED_FLAG_BITS 0b1111	

class Stepper {

private:
	class Command {
		Command *_next;
		uint8_t _count;
		uint8_t _feed;
		uint8_t _flags;

	public:
		Command(uint8_t count, uint8_t feed, uint8_t flags) {
			_count = count;
			_feed = feed;
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
			//return _count;
		}

		uint8_t getCount() {
			return _count;
		}

		uint8_t getFeed() {
			return (_feed);
		}

		uint8_t getFlags() {
			return _flags;
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

public:

	Stepper(int8_t stepPin, int8_t directionPin) {
		_directionPin = directionPin;
		_stepPin = stepPin;
		_head = NULL;
	}

	void addCommand(uint8_t count, uint8_t flags) {
		class Command *command = new Command(count, flags);
		if (!_head)
			_head = command;
		else
			_head->add(command);
	}

	bool step() {
		if (_head) {
			if (digitalRead(_stepPin)) {
				digitalWrite(_stepPin, LOW);
				if (head->getCount() <= 0)
					return true;
			} 
			else {
				digitalWrite(_stepPin, HIGH);
				_head->step();
			}			
		}
		return false;
	}

	bool next() {
		Command *head = _head;
		if (head) {

			_head = head->next();
			delete head;

			if (_head)
				digitalWrite(_directionPin, getDirection());

			return true;
		}
		return false;
	}

	uint8_t getFeed() {
		return (_head) ? _head->getFeed() : 1;
	}

	uint8_t getCount() {
		return (_head) ? _head->getMax() : 255;
	}
};

uint8_t _timer;
uint16_t _timeOutCount;

uint16_t _m1Count, _m2Count, _m1Max, _m2Max;

bool _isTimer;
bool _isTransmission;
bool _dataRequested;

class Stepper *_middle, *_proximal;
bool _hasMiddle, _hasProximal;

void setup() 
{
	_timer = 0;
	_isTransmission = false;
	_dataRequested = false;

	Wire.begin(0x8);// join i2c bus with address #8
	Wire.onReceive(recieveData); // register event
	Wire.onRequest(masterRequest); // register event
	Serial.begin(115200);
	//Serial1.begin(115200);

	pinMode(M1_STEP_PIN, OUTPUT);
	pinMode(M2_STEP_PIN, OUTPUT);
	pinMode(M1_DIRECTION_PIN, OUTPUT);
	pinMode(M2_DIRECTION_PIN, OUTPUT);

	pinMode(MS1_PIN, OUTPUT);
	pinMode(MS2_PIN, OUTPUT);
	pinMode(MS3_PIN, OUTPUT);

	digitalWrite(MS1_PIN, LOW);
	digitalWrite(MS2_PIN, HIGH);
	digitalWrite(MS3_PIN, LOW);

	_m1CommandList = new CommandList(M1_STEP_PIN, M1_DIRECTION_PIN);
	_m2CommandList = new CommandList(M2_STEP_PIN, M2_DIRECTION_PIN);

	timerSetup();

}

ISR(TIMER1_COMPA_vect) {
	bool maxUpdate = false;

	_m1Count += OCR1A;
	_m2Count += OCR1A;

	if (_m1Count >= _m1Max) {
		_m1Count = 0;
		maxUpdate = _m1CommandList->loadCommand();
	}

	if (_m2Count >= _m2Max) {
		_m2Count = 0;
		maxUpdate |= _m2CommandList->loadCommand();
	}

	if (maxUpdate)
		updateMax();

	if (_m1CommandList->hasStep() || _m2CommandList->hasStep())
		OCR1A = min((_m1Max - _m1Count), (_m2Max - _m2Count));
	else
		timerStop();
}

void updateMax() {
	uint8_t m1Feed = _m1CommandList->getFeed();
	uint8_t m2Feed = _m2CommandList->getFeed();

	if (m1Feed == 0) m1Feed = 1;
	if (m2Feed == 0) m2Feed = 1;

	if (m1Feed < m2Feed) {
		_m1Max = BASE_COMPARE_REGISTER;
		_m2Max = (BASE_COMPARE_REGISTER / m1Feed) * m2Feed;
	}
	else {
		_m1Max = (BASE_COMPARE_REGISTER / m2Feed) * m1Feed;
		_m2Max = BASE_COMPARE_REGISTER;
	}

	Serial.print("m1Feed - ");
	Serial.println(m1Feed);

	Serial.print("m2Feed - ");
	Serial.println(m2Feed);
}

void timerSetup() {
	noInterrupts();						//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;

	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);	//CTC Mode with a 1024 prescaler	

	interrupts();						//Enable interrupts 	
}

void timerStart() {

	Serial.println("Starting Timer!");

	_isTimer = true;
	noInterrupts();						//Disable interrupts

	TIMSK1 |= _BV(OCIE1A);				//Output compare register A enabled

	updateMax();

	OCR1A = min(_m1Max, _m2Max);		//Output compare register value

	Serial.print("m1 Max Count - ");
	Serial.println(_m1Max);

	Serial.print("m2 Max Count - ");
	Serial.println(_m2Max);

	interrupts();						//Enable interrupts
}

void timerStop() {

	Serial.println("Stopping Timer!");

	_isTimer = false;
	noInterrupts();						//Disable interrupts
	TIMSK1 &= ~_BV(OCIE1A);				//Output compare register A Disable
	interrupts();						//Enable interrupts
}

void masterRequest() {
	Serial.println("Data Available!");

	if (!_isTransmission)
		Wire.write(true);
	else
		_dataRequested = true;

	
	_timeOutCount = 0;

	//if (Wire.available() <= 0)
	//	while (Wire.available() <= 0 && _timeOutCount < TIME_OUT) { _timeOutCount++; delay(1); }	
}

void recieveData(int size) {
	_isTransmission = true;

	while (Wire.available() > 0) {
		//Serial.println("Recieving - ");

		uint8_t flags = Wire.read();
		uint8_t count = Wire.read();		

		_m1CommandList->addCommand(count, flags);

		flags = Wire.read();
		count = Wire.read();		

		_m2CommandList->addCommand(count, flags);
	}

	if (!_isTimer)
		timerStart();

	//if (_dataRequested)
	//	Wire.write(true);

	_isTransmission = false;
}


void loop() {
}
