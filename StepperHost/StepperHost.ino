#include <inttypes.h>
#include <Wire.h>

#define FASTEST_STEP 5

#define BUTTON_PIN1 8
#define BUTTON_PIN2 9
#define BUTTON_PIN3 10
#define BUTTON_PIN4 11

#define DIRECTION_FLAG_BIT _BV(4)

#define START_OF_MESSAGE 1
#define END_OF_MESSAGE _BV(1)


uint8_t _bufferCount;

bool _isButton;

uint8_t _m1SlideCL[][2] = {
	{ 0b11010001, 0b1 }, { 0b11110001, 0b1 }, { 0b11010001, 0b1 }, { 0b11110001, 0b10 }, { 0b11010001, 0b1 }, 
	{ 0b11110001, 0b1 }, { 0b11010001, 0b1 }, { 0b11110001, 0b10 }, { 0b11010001, 0b1 }, { 0b11110001, 0b10 }
};

uint8_t _m2SlideCL[][2] = {
	{ 0b10110001, 0b1 }, { 0b10110001, 0b1 }, { 0b10110001, 0b1 }, { 0b10110001, 0b10 }, { 0b10110001, 0b1 }, 
	{ 0b10110001, 0b1 }, { 0b10110001, 0b1 }, { 0b10110001, 0b10 }, { 0b10110001, 0b1 }, { 0b10110001, 0b10 }
};


//uint8_t _m1VertCL[][2] = { { 0b11110001, 0x2 }, { 0b11110001, 0x4 }, { 0b11110001, 0x6 } };
//uint8_t _m2VertCL[][2] = { { 0b10110010, 0x1 }, { 0b10110010, 0x2 }, { 0b10110010, 0x3 } };
//uint8_t _m1VertCL[][2] = { { 0b11110001, 10 } };
//uint8_t _m2VertCL[][2] = { { 0b10110010, 5 } };

uint8_t _m1VertCL[][2] = { { 0b11110010, 5 }, { 0b11110001, 10 } };
uint8_t _m2VertCL[][2] = { { 0b10110001, 10 }, { 0b10110010, 5 } };


void setup() 
{

	pinMode(0, INPUT);           // set pin to input
	digitalWrite(0, HIGH);       // turn on pullup resistors

	Serial.begin(115200);
	Serial1.begin(115200);
	Wire.begin();
	//Serial1.begin(115200);
	//Serial1.begin(speed, config) 

	_bufferCount = 0;
	_isButton = false;

	


	//for (int i = 0; i < 1; ++i)
		//serialSendData(_m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2);
	//for (int i = 0; i < 5; ++i)
	//	sendData(8, true, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 right turn
	
	
	//sendData(8, false, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 right turn
	//sendData(8, true, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 right turn
	//sendData(8, false, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 right turn
	/*

	delay(750);

	sendData(8, true, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide up

	delay(375);

	sendData(8, false, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide down

	delay(1000);

	sendData(8, true, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide up

	delay(675);

	sendData(8, false, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 left turn

	delay(750);

	sendData(8, false, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide down

	delay(750);

	sendData(8, true, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide up

	delay(375);

	sendData(8, false, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide down

	delay(1000);

	sendData(8, true, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 right turn

	delay(750);

	sendData(8, false, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 left turn
	*/

}

void testSend() {
	//for (int i = 0; i < 12; ++i)
	serialSendData(_m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2);

	//serialSendData(_m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2); //Slide up
}

void requestResponse(uint8_t addr, uint8_t size) {
	Wire.requestFrom(addr, size);

	while (Wire.available()) {
		Wire.read();
	}
}

void writeData(uint8_t addr, uint8_t mask, uint8_t m1Data[2], uint8_t m2Data[2]) {

	Wire.write(m1Data[0] ^ mask);
	Wire.write(m1Data[1]);
	Wire.write(m2Data[0] ^ mask);
	Wire.write(m2Data[1]);

	_bufferCount += 4;
	if (_bufferCount >= 32) {
		Wire.endTransmission(addr);
		//requestResponse(addr, 1);
		_bufferCount = 0;
		Wire.beginTransmission(addr);
	}
}

void writeSerialData(uint8_t m1Data[2], uint8_t m2Data[2]) {

	Serial1.write(m1Data[0]);
	Serial1.write(m1Data[1]);
	Serial1.write(m2Data[0]);
	Serial1.write(m2Data[1]);

}

void serialSendData(uint8_t m1Data[][2], uint8_t m2Data[][2], uint8_t count) {

	//Initiate a new Transmission
	Serial1.write(START_OF_MESSAGE);

	Serial.println("Sending Message!!");

	bool isBufferFull = false;
	int tCount = 0;

	int i = 0;

	while (i < count){
	    if (Serial1.availableForWrite() > 9) {
	    	if (isBufferFull) {
	    		Serial.println("Buffer Free, Sending!!");
	    		isBufferFull = false;
	    	}
	    	tCount += 4;
			writeSerialData(m1Data[i], m2Data[i]);
			i++;
		} else {
			if (!isBufferFull)
				Serial.println("Buffer Full! Waiting!!");
			isBufferFull = true;
			delayMicroseconds(500);
		}
	}
	//End a new Transmission
	Serial1.write(END_OF_MESSAGE);
	Serial.print("Transfer Finished, Bytes Sent: ");
	Serial.println(tCount);
	
}

void sendData(uint8_t addr, bool direction, uint8_t m1Data[][2], uint8_t m2Data[][2], uint8_t count) {

	Wire.beginTransmission(addr);

	//requestResponse(addr, 1);

	if (direction)
		for (int i = 0; i < count; ++i) {
			writeData(addr, 0, m1Data[i], m2Data[i]);
		}
	else
		for (int i = count - 1; i >= 0; i--) {
			writeData(addr, DIRECTION_FLAG_BIT, m1Data[i], m2Data[i]);
		}

	
	Wire.endTransmission(addr);
}

void loop() {

	bool buttonPin1 = digitalRead(BUTTON_PIN1);
	bool buttonPin2 = digitalRead(BUTTON_PIN2);
	bool buttonPin3 = digitalRead(BUTTON_PIN3);
	bool buttonPin4 = digitalRead(BUTTON_PIN4);

	if (buttonPin1 && !_isButton) {
		//sendData(8, true, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2);
		//_isButton = true;
	}			
	if (buttonPin2 && !_isButton) {
		//sendData(8, false, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2);
		//_isButton = true;
	}
	if (buttonPin3 && !_isButton) {
		testSend();
		_isButton = true;
	}
	if (buttonPin4 && !_isButton) {
		//sendData(8, true, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2);
		//_isButton = true;
	}
	if (!buttonPin1 && !buttonPin2 && !buttonPin3 && !buttonPin4) {
		_isButton = false;
	}

	delay(250);
}
