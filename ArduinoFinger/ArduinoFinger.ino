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

#define COMMAND_FLAG _BV(7)		

void setup() {
	Serial.begin(9600);
	controllerSetup();


}/*
 * I ran into a lot of issues trying to nail down a
 * reliable communication protocol.
 * Keeping the arduino's serial communications up with 
 * the speed required for 1/16th microsteps was a real pain.
 * I found the I could easily crash the 
 * Using a handshaking approach, the Arduino will respond when
 * it is ready to recieve more data.
 */

void loop() {	
	uint8_t length = Serial.available();
	if (length > 0) {
		uint8_t input[length];
		Serial.readBytes(input, length);				//Reading all data available in one chunk	
		int i = 0;										
		while(isFingerReady() && i < length) {			//Loops through inputted data, pausing if command list is full
			uint16_t flags = input[i++];
			if ((flags & COMMAND_FLAG) > 0) {
				unsigned short feed = (input[i++] << 8);
				feed |= input[i++];
				addToQueue(flags, feed);
			}
		}
		Serial.write(1);
	}	
}
