#include "SDPArduino.h"
#include <Wire.h>
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200
/*
   Joel Hutton
   this script is for testing how to make the distance move given a distance and angle
 */


/*
   Motor mapping

   0 - Left Reversed
   1 - Right Reversed
   2 - Middle to left
   3 - kicker
   4 - flippers
   5 - unnasigned
 */

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};

void updateMotorPositions() {
	// Request motor position deltas from rotary slave board
	Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);

	// Update the recorded motor positions
	for (int i = 0; i < ROTARY_COUNT; i++) {
		positions[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
	}
}

void printMotorPositions() {
	Serial.print("Motor positions: ");
	for (int i = 0; i < ROTARY_COUNT; i++) {
		Serial.print(positions[i]);
		Serial.print(' ');
	}
	Serial.println();
	delay(PRINT_DELAY);  // Delay to avoid flooding serial out
}

void go(int degrees, bool forwards){
	bool finished = false;
	int target[]={positions[0]+degrees,positions[1]+degrees}; 
	int start[]={positions[0],positions[1]}; 
	if(forwards){
		Serial.write("forwards\r\n");
		int delta0;
		int delta1;
		while(!finished){
			updateMotorPositions();
			delta0=positions[0]-target[0];
			delta1=positions[1]-target[1];
			if(abs(delta0)<=5 && abs(delta1)<=5){
				finished = true;
			}
			else if(delta0-delta1>5){
				motorBackward(0,80);	
				motorBackward(1,100);	
			}
			else if(delta1-delta0>5){
				motorBackward(0,100);	
				motorBackward(1,80);	
			}
			else{
				motorBackward(0, 100);
				motorBackward(1, 100);
			}
		}
	}
	else{
		Serial.write("backwards\r\n");
		int delta0;
		int delta1;
		while(!finished){
			updateMotorPositions();
			delta0=positions[0]-target[0];
			delta1=positions[1]-target[1];
			if(abs(delta0)<=5 && abs(delta1)<=5){
				finished = true;
			}
			else if(delta0-delta1>5){
				motorForward(0,80);	
				motorForward(1,100);	
			}
			else if(delta1-delta0>5){
				motorForward(0,100);	
				motorForward(1,80);	
			}
			else{
				motorForward(0, 100);
				motorForward(1, 100);
			}
		}
	}
	motorAllStop();
}

//Don't ask why, I don't know 
float degToMetre = 1600;
void goCalibrate(){
	Serial.write("distance calibration:\r\n");
	char ser=Serial.read();
	//quit signal
	int delta;
	bool forwards=true;
	while(ser!='q'){
		delta=0;
		switch(ser){
			//Roman Numerals because I'm a horrible back
			case 'B':
				forwards=false;
				break;
			case 'F':
				forwards=true;
				break;
			case 'I':
				delta=1;
				break;
			case 'V':
				delta=5;
				break;
			case 'X':
				delta=10;
				break;
			case 'L':
				delta=50;
				break;
			case 'i':
				delta=-1;
				break;
			case 'v':
				delta=-5;
				break;
			case 'x':
				delta=-10;
				break;
			case 'l':
				delta=-50;
				break;
			case 'k':
				Serial.write("going for:");
				Serial.print(degToMetre);
				Serial.write("\r\n");
				go(degToMetre, forwards);
				forwards = !forwards;
				break;
		}
		degToMetre+=delta;
		if(delta !=0){
			Serial.print("degrees Travelled:");
			Serial.print(degToMetre);
			Serial.write("\r\n");
		}
		ser=Serial.read();
	}
}

void setup() {
	digitalWrite(8, HIGH);  // Radio on
	Serial.begin(115200);  // Serial at given baudrate
	Wire.begin();  // Master of the I2C bus
	SDPsetup();
	helloWorld();
}

void loop(){
	goCalibrate();	
	char input;
	input = Serial.read();
	switch (input) {
		//go forwards
		case 'c':
			go(1000,true);
			break;
			//go antiforwards
		case 'a':
			go(1000,false);
			break;
		default:
			break;
	}
}

