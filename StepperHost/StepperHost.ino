#include <inttypes.h>
#include <Wire.h>

#define FASTEST_STEP 5

#define BUTTON_PIN1 8
#define BUTTON_PIN2 9
#define BUTTON_PIN3 10
#define BUTTON_PIN4 11

#define DIRECTION_FLAG_BIT _BV(4)


uint8_t _bufferCount;

bool _isButton;

uint8_t _m1SlideCL[][2] = {
	{ 0b010001, 0b0 }, { 0b110001, 0b1 }, { 0b010001, 0b0 }, { 0b110001, 0b10 }, { 0b010001, 0b0 }, 
	{ 0b110001, 0b1 }, { 0b010001, 0b0 }, { 0b110001, 0b10 }, { 0b010001, 0b0 }, { 0b110001, 0b10 }, 
	{ 0b010001, 0b0 }, { 0b110001, 0b10 }, { 0b010001, 0b0 }, { 0b110001, 0b1 }, { 0b010001, 0b0 }, 
	{ 0b110001, 0b10 }, { 0b010001, 0b0 }, { 0b110001, 0b10 }, { 0b010001, 0b0 }, { 0b110001, 0b10 }, 
	{ 0b010001, 0b0 }, { 0b110001, 0b10 }, { 0b010001, 0b0 }, { 0b110001, 0b11 }, { 0b010001, 0b0 }, 
	{ 0b110001, 0b10 }, { 0b010001, 0b0 }, { 0b110001, 0b11 }, { 0b010001, 0b0 }, { 0b110001, 0b101 }, 
	{ 0b010001, 0b0 }, { 0b110001, 0b10001 }, { 0b110001, 0b10 }
};

uint8_t _m2SlideCL[][2] = {
	{ 0b110000, 0b1 }, { 0b110001, 0b1 }, { 0b110000, 0b1 }, { 0b110001, 0b10 }, { 0b110000, 0b1 }, 
	{ 0b110001, 0b1 }, { 0b110000, 0b1 }, { 0b110001, 0b10 }, { 0b110000, 0b1 }, { 0b110001, 0b10 }, 
	{ 0b110000, 0b1 }, { 0b110001, 0b10 }, { 0b110000, 0b1 }, { 0b110001, 0b1 }, { 0b110000, 0b1 }, 
	{ 0b110001, 0b10 }, { 0b110000, 0b1 }, { 0b110001, 0b10 }, { 0b110000, 0b1 }, { 0b110001, 0b10 }, 
	{ 0b110000, 0b1 }, { 0b110001, 0b10 }, { 0b110000, 0b1 }, { 0b110001, 0b11 }, { 0b110000, 0b1 }, 
	{ 0b110001, 0b10 }, { 0b110000, 0b1 }, { 0b110001, 0b11 }, { 0b110000, 0b1 }, { 0b110001, 0b101 }, 
	{ 0b110000, 0b1 }, { 0b110001, 0b10001 }, { 0b110010, 0b1 }
};



uint8_t _m1VertCL[][2] = { { 0b100001, 0xC8 } };
uint8_t _m2VertCL[][2] = { { 0b110001, 0xC8 } };


void setup() 
{
	Serial.begin(115200);
	Wire.begin();
	//Serial1.begin(115200);
	//Serial1.begin(speed, config) 

	_bufferCount = 0;
	_isButton = false;

	delay(1500);

	sendData(8, true, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2); //90 right turn

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

}

void requestResponse(uint8_t addr, uint8_t size) {
	Wire.requestFrom(addr, size);

	while (Wire.available()) {
		Serial.print("Recieved: ");
		Serial.println(Wire.read(),2);
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
		Serial.println("Buffer Full!");
		requestResponse(addr, 1);
		Serial.println("Buffer Clear!");
		_bufferCount = 0;
		Wire.beginTransmission(addr);
	}
}

void sendData(uint8_t addr, bool direction, uint8_t m1Data[][2], uint8_t m2Data[][2], uint8_t count) {

	Wire.beginTransmission(addr);

	Serial.println("Requesting Response!");
	requestResponse(addr, 1);

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
		sendData(8, true, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2);
		_isButton = true;
		Serial.println("Button 1!");
	}			
	if (buttonPin2 && !_isButton) {
		sendData(8, false, _m1SlideCL, _m2SlideCL, sizeof(_m1SlideCL) / 2);
		_isButton = true;
		Serial.println("Button 2!");
	}
	if (buttonPin3 && !_isButton) {
		sendData(8, false, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2);
		_isButton = true;
		Serial.println("Button 3!");
	}
	if (buttonPin4 && !_isButton) {
		sendData(8, true, _m1VertCL, _m2VertCL, sizeof(_m1VertCL) / 2);
		_isButton = true;
		Serial.println("Button 4!");
	}
	if (!buttonPin1 && !buttonPin2 && !buttonPin3 && !buttonPin4) {
		_isButton = false;
	}

	delay(15);
}
