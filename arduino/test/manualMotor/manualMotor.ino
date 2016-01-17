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
  char input;
  input = Serial.read();

  switch (input) {
  case '8':
    motorBackward(0, 100);
    motorBackward(1, 100);
    break;
    
  case '2':
    motorForward(0, 100);
    motorForward(1, 100);
    break;

case '5':
  motorStop(0);
   motorStop(1);
   motorStop(2);
  break;
  
  
  
case '6':
    motorBackward(2, 100);
  break;
case '4':
    motorForward(2, 100);
  break;
  
case '9':
    motorBackward(0, 100);
    motorForward(1, 50);
    motorBackward(2, 100);
  break;
case '7':
    motorForward(0, 50);
    motorBackward(1, 100);
    motorForward(2, 100);
  break;
  
case '3':
    motorBackward(0, 50);
    motorForward(1, 100);
    motorBackward(0, 50);
    motorBackward(2, 100);
  break;
case '1':
    motorForward(0, 100);
    motorBackward(1, 50);
    motorForward(2, 100);
  break;
  
  //right flipper open
case 'k':
    motorForward(5, 80);
    delay(300);
    motorStop(5);
  break;
  
  //right flipper close
case 'l':
    motorBackward(5, 80);
    delay(300);
    motorStop(5);
  break;
  
  
  
  //left flipper open
case 'a':
    motorForward(4, 80);
    delay(300);
    motorStop(4);
  break;
  
  //left flipper close
case 's':
    motorBackward(4, 80);
    delay(300);
    motorStop(4);
  break;
  
  
  
  //kicker down
case 'h':
    motorForward(3, 100);
    delay(300);
    motorStop(3);
  break;
  
  // kick
case 'y':
    motorBackward(3, 100);
    delay(300);
    motorForward(3, 100);
    delay(100);
    motorStop(3);
  break;
  }


}

