/**
* Transmit button status to computer from
* Arduino
*/

//#include "SDPArduino.h"
//#include <Wire.h>

//analog in/digital I/O ports A0 to A3

const int buttonPin1 = A2;
const int buttonPin2 = A3;
//const int buttonPin3 = 3;
//const int buttonPin4 = 4;

void setup() {
  //SDPsetup();
  Serial.begin(9600);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  //pinMode(buttonPin3, INPUT);
  //pinMode(buttonPin4, INPUT);
}

void loop() {
  if(digitalRead(buttonPin1) == HIGH){
    	Serial.println("Sensor 1");
  }

  if(digitalRead(buttonPin2) == HIGH){
	Serial.println("Sensor 2");
  }

  //if(digitalRead(buttonPin3) == HIGH){
 //	Serial.println("Sensor 3");
  //}

  //if(digitalRead(buttonPin4) == HIGH){
//	Serial.println("Sensor 4");
 // }


//OR if you want all button statuses regardless if pushed/not pushed

//Serial.println("Sensor 1 :");
//Serial.print(digitalRead(buttonPin1));
//Serial.println("Sensor 2 :");
//Serial.print(digitalRead(buttonPin2));
//Serial.println("Sensor 3 :");
//Serial.print(digitalRead(buttonPin3));
//Serial.println("Sensor 4 :");
//Serial.print(digitalRead(buttonPin4));

  delay(100); // 100ms delay
}
