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
void setup(){
	SDPsetup();
	helloWorld();
}

void turn(int time, bool clockwise){
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
}

//Don't ask why, I don't know 
float fullRotAc = 1650;
float fullRotCw = 1600;
void turnCalibrate(){
	fullRotAc = 1500;
	fullRotCw = 1500;
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
				turn(time, adjustCw);
				adjustCw=!adjustCw;
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

