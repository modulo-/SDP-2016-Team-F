#include "SDPArduino.h"
#include <Wire.h>
int i = 0;


/*
   Motor mapping

   0 - Left Reversed
   1 - Right Reversed
   2 - Middle to left
   3 - kicker
   4 - left flipper
   5 - right flipper
 */
void setup(){
	SDPsetup();
	helloWorld();
}


//do the calibration for 50cm, 100cm and 150cm
//50 approx 100 130
//100 approx 100 150
//a50 approx 100 173 
void kickerCalibration(){
	Serial.write("kicker calibration:\r\n");
	int kickerStrength=100;
	int kickerTime=150;
	char ser=Serial.read();
	//quit signal
	int delta;
	bool adjustTime=true;
	while(ser!='q'){
		delta=0;
		switch(ser){
			//Roman Numerals because I'm a horrible back
			case 'T':
				adjustTime=true;
				break;
			case 'P':
				adjustTime=false;
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
			  //close flippers
			  motorBackward(5, 60);
			  //move kicker back out way
			  delay(800);
			  //move flippers away
			  motorForward(5, 80);
			  delay(800);
			  motorAllStop();
			  delay(400);

			  Serial.write("kicking at P:");
			  Serial.print(kickerStrength);
			  Serial.write(" T:");
			  Serial.print(kickerTime);
			  Serial.write("\r\n");
			  motorBackward(3, kickerStrength);
			  delay(kickerTime);
			  motorForward(3, 30);
			  delay(1000);
			  motorStop(3);
			break;
		}
		if(adjustTime){
			kickerTime+=delta;
		}
		else{
			kickerStrength+=delta;
		}
		if(delta !=0){
		Serial.write("Power:");
		Serial.print(kickerStrength);
		Serial.print(" Time:");
		Serial.print(kickerTime);
		Serial.write("\r\n");
		}
		ser=Serial.read();
	}
}

void loop(){
	char input;
	kickerCalibration();
}

