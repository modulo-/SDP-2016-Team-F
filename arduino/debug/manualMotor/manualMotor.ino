#include "SDPArduino.h"
#include <Wire.h>
int i = 0;


/*
Motor mapping
 0

 0 - Left motor (polarity reversed)
 1 - Right motor (polarity reversed)
 2 - Middle motor (left is 'forward')
 3 - kicker
 4 - flippers
 5 - nothing
 */

void setup() {
  SDPsetup();
  helloWorld();
}

void loop() {
  react();
}

void react() {
char input;
  input = Serial.read();
  switch (input) {
    case '8':
      Serial.println("forward");
      motorBackward(0, 100);
      motorBackward(1, 100);
      motorStop(2);
      break;

    case '2':
      Serial.println("backward");
      motorForward(0, 100);
      motorForward(1, 100);
      motorStop(2);
      break;

    case '5':
      Serial.println("stop");
      motorStop(0);
      motorStop(1);
      motorStop(2);
      break;

    case '6':
      Serial.println("right");
      motorForward(0, 0);
      motorBackward(1,65);
      motorBackward(2,100);
      break;

    case '4':
      Serial.println("left");
      motorBackward(0,65);
      motorForward(1, 0);
      motorForward(2, 100);
      break;

    case '9':
      Serial.println("forwards clockwise");
      motorBackward(0, 100);
      motorForward(1, 50);
      motorBackward(2, 100);
      break;

    case '7':
      Serial.println("forwards anticlockwise");
      motorForward(0, 50);
      motorBackward(1, 100);
      motorForward(2, 100);
      break;

    case '3':
      Serial.println("backwards clockwise");
      motorBackward(0, 50);
      motorForward(1, 100);
      motorBackward(0, 50);
      motorBackward(2, 100);
      break;

    case '1':
      Serial.println("backwards anticlockwise");
      motorForward(0, 100);
      motorBackward(1, 50);
      motorForward(2, 100);
      break;

    //flippers open
    case 'k':
      Serial.write("flippers open\r\n");
      motorForward(5, 80);
      delay(300);
      motorStop(5);
      break;

    //flippers close
<<<<<<< HEAD
    case 'l':
      Serial.write("flippers close\r\n");
      motorBackward(5, 80);
      delay(300);
      motorStop(5);
      break;

//    //left flipper open
//    case 'a':
//      Serial.println("left flipper open");
//      motorForward(4, 80);
//      delay(300);
//      motorStop(4);
//      break;
//
//    //left flipper close
//    case 's':
//      Serial.println("left flipper close");
//      motorBackward(4, 80);
//      delay(300);
//      motorStop(4);
//      break;

<<<<<<< HEAD
    //kick
    case 'h':
      Serial.write("kicker down\r\n");
      motorForward(3, 100);
      delay(300);
      motorStop(3);
      break;

    // kicker down
    case 'j':
      Serial.write("kick\r\n");
      motorBackward(3, 30);
    
    // kick
    case 'y':
      Serial.println("kick");
      motorBackward(3, 100);
      delay(300);
      motorForward(3, 30);
      delay(100);
      motorStop(3);
      break;
  }
}
