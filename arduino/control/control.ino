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

void loop(){
}

