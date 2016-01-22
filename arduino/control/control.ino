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

//checksum for the comms data packet
//b0 is XOR of the even characters, 
//b1 is XOR of the odd characters. 
byte* dataCheckSum(byte* data,int length){
	byte* b0;
	byte* b1;
	b0 = (byte*) malloc(sizeof(byte));
	b1 = (byte*) b0 + sizeof(byte);
	*(b0) = 0x0;
	*(b1) = 0x0;
	for(int i =0; i<length;i++){
		if(i%2==0){
			*(b0) = *(b0) ^data[i];
		}		
		else{
			*(b1) = *(b1) ^ data[i];
		}
	}
}

//do the calibration for 50cm, 100cm and 150cm
//50 approx 100 130
//100 approx 100 150
//a50 approx 100 
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
	input = Serial.read();
	switch (input) {
		case '8':
			Serial.write("forward\r\n");
			motorBackward(0, 100);
			motorBackward(1, 100);
			break;

		case '2':
			Serial.write("backward\r\n");
			motorForward(0, 100);
			motorForward(1, 100);
			break;

		case '5':
			Serial.write("stop\r\n");
			motorStop(0);
			motorStop(1);
			motorStop(2);
			break;

		case '6':
			Serial.write("right\r\n");
			motorBackward(2, 100);
			break;
		case '4':
			Serial.write("left\r\n");
			motorForward(2, 100);
			break;

		case '9':
			Serial.write("forwards clockwise\r\n");
			motorBackward(0, 100);
			motorForward(1, 50);
			motorBackward(2, 100);
			break;
		case '7':
			Serial.write("forwards anticlockwise\r\n");
			motorForward(0, 50);
			motorBackward(1, 100);
			motorForward(2, 100);
			break;

		case '3':
			Serial.write("backwards clockwise\r\n");
			motorBackward(0, 50);
			motorForward(1, 100);
			motorBackward(0, 50);
			motorBackward(2, 100);
			break;
		case '1':
			Serial.write("backwards anticlockwise\r\n");
			motorForward(0, 100);
			motorBackward(1, 50);
			motorForward(2, 100);
			break;

			//right flipper open
		case 'k':
			Serial.write("right flipper open\r\n");
			motorForward(5, 80);
			delay(300);
			motorStop(5);
			break;

			//right flipper close
		case 'l':
			Serial.write("right flipper close\r\n");
			motorBackward(5, 80);
			delay(300);
			motorStop(5);
			break;



			//left flipper open
		case 'a':
			Serial.write("left flipper open\r\n");
			motorForward(4, 80);
			delay(300);
			motorStop(4);
			break;

			//left flipper close
		case 's':
			Serial.write("left flipper close\r\n");
			motorBackward(4, 80);
			delay(300);
			motorStop(4);
			break;



			//kicker down
		case 'h':
			Serial.write("kicker down\r\n");
			motorForward(3, 100);
			delay(300);
			motorStop(3);
			break;

			// kick
		case 'y':
			Serial.write("kick\r\n");
			motorBackward(3, 100);
			delay(300);
			motorForward(3, 100);
			delay(100);
			motorStop(3);
			break;
		default:
			break;
	}
}

