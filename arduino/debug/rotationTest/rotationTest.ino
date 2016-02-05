#include "SDPArduino.h"
#include <Wire.h>
/*
   Joel Hutton
   this script is for testing how to rotate the robot by a given angle
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
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};

void setup() {
	digitalWrite(8, HIGH);  // Radio on
	Serial.begin(115200);  // Serial at given baudrate
	Wire.begin();  // Master of the I2C bus
	SDPsetup();
	helloWorld();
}

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

void turn(int time, bool clockwise){
	updateMotorPositions();
	printMotorPositions();
	int start0=positions[0];
	int start1=positions[1];
	if(clockwise){
		Serial.write("clockwise\r\n");
		motorBackward(0, 100);
		motorForward(1, 100);
		motorBackward(2,100);
		//delay(fullRotCw*(angle/360.0));
	}
	else{
		Serial.write("anti-clockwise\r\n");
		motorForward(0, 100);
		motorBackward(1, 100);
		motorForward(2,	100);
		//delay(fullRotAc*(angle/360.0));
	}
	delay(time);
	motorAllStop();
	updateMotorPositions();
	printMotorPositions();
	Serial.print("Left turned:");
	Serial.print(positions[0]-start0);
	Serial.print("Right turned:");
	Serial.print(positions[1]-start1);
	Serial.print("\r\n");
}

void turnDeg(int degrees, bool cw){
	bool finished = false;
	//everything is confusing because the motors are mounted backwards
	if(cw)degrees=-degrees;
	updateMotorPositions();
	int start[]={positions[0],positions[1]}; 
	Serial.write("forwards\r\n");
	int delta0=0;
	int delta1=0;
	int leftPower=100;
	int rightPower=100;
	int prevTime=millis();
	bool timeout;
	Serial.print("Starting at: ");
	Serial.print(start[0]);
	Serial.print(" ");
	Serial.print(start[1]);
	Serial.print(" Going to:");
	Serial.print(positions[0]+degrees);
	Serial.print(" ");
	Serial.print(positions[1]-degrees);
	Serial.print("\r\n");
	while(!finished){
		updateMotorPositions();
		timeout=false;
		//timeout = (millis()-prevTime) > 1000;
		if(timeout){	
			prevTime=millis();
			printMotorPositions();
		}
		delta0=-(positions[0]-start[0]);
		delta1=(positions[1]-start[1]);
		if(!cw){
			delta0=-delta0;
			delta1=-delta1;
		}
		if(leftPower!=0&& abs(delta0)>=abs(degrees)){
			Serial.print("Left finished\r\n");
			leftPower=0;
		}
		if(rightPower!=0&& abs(delta1)>=abs(degrees)){
			Serial.print("Right finished\r\n");
			rightPower=0;
		}
		if((abs(delta0)>=abs(degrees)&&abs(delta1)>=abs(degrees)) || (leftPower==0 && rightPower==0)){
			finished = true;
			break;
		}
		else if(delta0-delta1>5){
			if(timeout){
				Serial.print("Too far right, reducing power to left engine\r\n");
				Serial.print(" Left has gone:");
				Serial.print(delta0);
				Serial.print(" Right has gone:");
				Serial.print(delta1);
				Serial.print("\r\n");
			}
			if(leftPower>90)leftPower=90;
		}
		else if(delta1-delta0>5){
			if(timeout){
				Serial.print("Too far left, reducing power to right engine\r\n");
				Serial.print("Left has gone:");
				Serial.print(delta0);
				Serial.print(" Right has gone:");
				Serial.print(delta1);
				Serial.print("\r\n");
			}
			if(rightPower>90)rightPower=90;
		}
		else{
			if(timeout)Serial.print("Going straight\r\n");
			if(leftPower!=0)leftPower=100;
			if(rightPower!=0)rightPower=100;
		}
		if(cw){
			motorBackward(0,leftPower);
			motorForward(1,rightPower);
		}
		else{
			motorForward(0,leftPower);
			motorBackward(1,rightPower);
		}
	}
	motorAllStop();
	updateMotorPositions();
  printMotorPositions();
	Serial.print("Finished at: ");
	Serial.print(positions[0]);
	Serial.print(" ");
	Serial.print(positions[1]);
	Serial.print("\r\n");
}

//Don't ask why, I don't know 
float fullRotAc = 550;
float fullRotCw = 550;
void turnCalibrate(){
	Serial.write("turning calibration:\r\n");
	char ser=Serial.read();
	//quit signal
	int delta;
	bool adjustCw=true;
	while(ser!='q'){
		delta=0;
		switch(ser){
			//Roman Numerals because I'm a horrible back
			case 'A':
				adjustCw=false;
				break;
			case 'C':
				adjustCw=true;
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
				Serial.write("Turning for:");
				if(adjustCw){
					Serial.print(fullRotCw);
				}
				else{
					Serial.print(fullRotCw);
				}
				Serial.write("\r\n");
				int time = adjustCw? fullRotCw : fullRotAc;
				turnDeg(time, adjustCw);
				break;
		}
		if(adjustCw){
			fullRotCw+=delta;
		}
		else{
			fullRotAc+=delta;
		}
		if(delta !=0){
			Serial.write("Ac Time:");
			Serial.print(fullRotAc);
			Serial.print("Cw Time:");
			Serial.print(fullRotCw);
			Serial.write("\r\n");
		}
		ser=Serial.read();
	}
}

void loop(){
	turnCalibrate();	
	char input;
	input = Serial.read();
	switch (input) {
		//turn clockwise
		case 'c':
			turn(1000,true);
			break;
			//turn anticlockwise
		case 'a':
			turn(1000,false);
			break;
		default:
			break;
	}
}

