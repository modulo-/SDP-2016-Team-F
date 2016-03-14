/*
  Group 14 Arduino Code
*/

#include <Wire.h> 
#include <avr/wdt.h>

#include "SDPArduino.h"

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};

void updateMotorPositions() {
  // Request motor position deltas from rotary slave board
  Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);
  int8_t values[ROTARY_COUNT];
  //check for error
  bool error=true;
  for (int i = 0; i < ROTARY_COUNT; i++) {
    int8_t a =(int8_t) Wire.read();  // Must cast to signed 8-bit type
    values[i]=a;
    if(a!=-1){error=false;}
  }
  if(error){
    Serial.println("cannot communicate with encoder board");
  }
  else{
      // Update the recorded motor positions
      for (int i = 0; i < ROTARY_COUNT; i++) {
        positions[i] += values[i]; 
      }
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

byte cmd, data, response;
unsigned long current_millis;

int rotary_positions[6] = {0, 0, 0, 0, 0, 0};

// For timing purposes
unsigned long current_micros;
unsigned long old_micros;
unsigned long old_millis;

byte max_time = 0;    // the maximum time (in ms)
byte min_time = 255;  // the minimum time (in us)

int targetHeading, headingDiff;

int turnPower;
int strafePower;

#define _HEADING_TOLERANCE 1
#define _HEADING_CORRECTION_COEFFICIENT 0.5
#define _TURN_DELAY 50

#define _RAD_TO_DEG 57.2957795

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6



/*
  Setup function. Performs the standard SDPsetup routine supplied by
  the course staff. Also sets up the kicker servo.
*/



void setup() {

  digitalWrite(8, HIGH);  // Radio on
  Serial.begin(115200);  // Serial at given baudrate
  Wire.begin();  // Master of the I2C bus

  Wire.begin(ROTARY_SLAVE_ADDRESS);  // I2C slave at given address

  SDPsetup();
  // initialise the IMU

  //Initialise the WDT and flush the serial...
  Serial.println("hello world");
  Serial.flush();
}

int prevPositions[ROTARY_COUNT];
int time=0;
void loop() {
  if(millis()-time>=200){
      updateMotorPositions();
      bool changed=false;
      for(int i=0; i<ROTARY_COUNT;i++){
          if(positions[i]!=prevPositions[i]){
            changed=true;
          }
          prevPositions[i]=positions[i];
          time=millis();
      }
      if(changed){ printMotorPositions(); }
  }
  react();
}

/*
  Main loop. Runs an endless loop of fetching commands from the serial
  comms, decoding them, and generally doing the stuff we're supposed to do.
*/


void returnRotaryPosition(byte data) {
  if (data >= 0 && data < 6) {
    /*
      if response is 0 then it won't get sent.
      Let's use a bias of 200 so that negative -> <200
      and positive -> >200.
    */
    response = (byte) (200 + rotary_positions[data]);
  }
}

void updateRotaryPositions() {
  // Request motor position deltas from rotary slave board
  Wire.requestFrom(ROTARY_SLAVE_ADDRESS, 6);

  // Update the recorded motor positions
  for (int i = 0; i < 6; i++) {
    rotary_positions[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
  }
}

void react() {
  char input;
  if(Serial.available()){
      int motorNumber=Serial.read()-'0' +0;
      Serial.println(motorNumber); 
      if(motorNumber >=0 && motorNumber <=5){
          motorForward(motorNumber, 100);
          Serial.println(motorNumber);
          delay(600);
          Serial.println("off");
          motorAllStop();
      }
    }
}
