/*
  Group 14 Arduino Code
*/

#include <Wire.h> 
#include <avr/wdt.h>

#include "SDPArduino.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200

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

/* Assign a unique ID to the sensors */

Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);


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
  initSensors();

  //Initialise the WDT and flush the serial...
  wdt_enable(WDTO_1S);
  Serial.flush();
}

void loop() {
  Serial.print("Heading :");
  Serial.println(getCurrentHeading());
  updateMotorPositions();
  printMotorPositions();
  react();
  //delay(200);
}


void initSensors()


{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
  }
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);

  /* Initialise the sensor */
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
  }
}

/*
  Main loop. Runs an endless loop of fetching commands from the serial
  comms, decoding them, and generally doing the stuff we're supposed to do.
*/



void returnHeading() {
  response = (byte) (getCurrentHeading() / 2);
}

void computeMinMaxTimings() {
  int timeDiffMicros = (current_micros - old_micros) / 10;
  if (timeDiffMicros > 255) {
    timeDiffMicros = 255;
  }
  if (timeDiffMicros == 0) {
    timeDiffMicros = 1;
  }
  int timeDiffMillis = (current_millis - old_millis) * 10;
  if (timeDiffMillis > 255) {
    timeDiffMillis = 255;
  }
  if (timeDiffMillis == 0) {
    timeDiffMillis = 1;
  }
  if (min_time > timeDiffMicros) min_time = (byte) timeDiffMicros;
  if (max_time < timeDiffMillis) max_time = (byte) timeDiffMillis;
}

void returnTiming(byte data) {
  if (!data) {
    int timeDiff = (current_micros - old_micros) / 10;
    if (timeDiff > 255) {
      timeDiff = 255;
    }
    if (timeDiff == 0) {
      timeDiff = 1;
    }
    response = (byte) timeDiff;
  }

  if (data == 1) {
    response = min_time;
  }
  else if (data == 2) {
    response = max_time;
  }
}

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

void returnBatteryVoltage() {
  int relativeVoltage = analogRead(2); // Input on A2.
  float absoluteVoltage = (5.0 * relativeVoltage) / 1023.0;
  /*
    Need to multiply by 2*10 since the potential divider on the board
    divided the voltage by 2, and we want a larger precision number.
    I.e. voltage will go from 0-100.
  */
  char responseVoltage = absoluteVoltage * 20;
  response = responseVoltage;
}

void updateRotaryPositions() {
  // Request motor position deltas from rotary slave board
  Wire.requestFrom(ROTARY_SLAVE_ADDRESS, 6);

  // Update the recorded motor positions
  for (int i = 0; i < 6; i++) {
    rotary_positions[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
  }
}

/*
  getHeadingDiff:

  Returns the relative difference between the target heading
  and the current heading. Positive direction is
  counter-clockwise, so a returned value of eg. 10 means that
  the robot should turn 10 degrees left to be on target.
  Similarly, a returned value of -20 would indicate that the
  robot would have to turn 20 degrees right to be on target.
*/

int getHeadingDiff(int targetHeading, int currentHeading) {
  int diff = (targetHeading - currentHeading + 360) % 360;
  if (diff >= 180) {
    return -360 + diff;
  }
  return diff;
}

int getCurrentHeading() {
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    return (int) orientation.heading;
  }
}

byte getHeadingTolerance(int currentHeading, int targetHeading) {
  int deltaAngle;
  deltaAngle = getHeadingDiff(targetHeading, currentHeading);
  return 5 + abs(deltaAngle) / 10;
}

/*
  Returns degrees per second.
*/

int getGyroOrientation() {
  sensors_event_t event;
  gyro.getEvent(&event);

  return (int) (event.gyro.y * _RAD_TO_DEG);
}

void react() {
  char input;
  input = Serial.read();
  switch (input) {
    case '8':
      Serial.write("forward\n");
      motorBackward(0, 100);
      motorBackward(1, 100);
      motorStop(2);
      break;

    case '2':
      Serial.write("backward\n");
      motorForward(0, 100);
      motorForward(1, 100);
      motorStop(2);
      break;

    case '5':
      Serial.write("stop\n");
      motorStop(0);
      motorStop(1);
      motorStop(2);
      break;

    case '6':
      Serial.write("right\n");
      motorForward(0, 0);
      motorBackward(1, 85);
      motorBackward(2, 100);
      break;

    case '4':
      Serial.write("left\n");
      motorBackward(0, 85);
      motorForward(1, 0);
      motorForward(2, 100);
      break;

    case '9':
      Serial.write("forwards clockwise\n");
      motorBackward(0, 100);
      motorForward(1, 50);
      motorBackward(2, 100);
      break;

    case '7':
      Serial.write("forwards anticlockwise\n");
      motorForward(0, 50);
      motorBackward(1, 100);
      motorForward(2, 100);
      break;

    case '3':
      Serial.write("backwards clockwise\n");
      motorBackward(0, 50);
      motorForward(1, 100);
      motorBackward(0, 50);
      motorBackward(2, 100);
      break;

    case '1':
      Serial.write("backwards anticlockwise\n");
      motorForward(0, 100);
      motorBackward(1, 50);
      motorForward(2, 100);
      break;

    //flippers open
    case 'k':
      Serial.write("flippers open\n");
      motorForward(5, 80);
      delay(300);
      motorStop(5);
      break;

    //flippers close
    case 'l':
      Serial.write("flippers close\n");
      motorBackward(5, 80);
      delay(300);
      motorStop(5);
      break;

    //    //left flipper open
    //    case 'a':
    //      Serial.write("left flipper open\n");
    //      motorForward(4, 80);
    //      delay(300);
    //      motorStop(4);
    //      break;
    //
    //    //left flipper close
    //    case 's':
    //      Serial.write("left flipper close\n");
    //      motorBackward(4, 80);
    //      delay(300);
    //      motorStop(4);
    //      break;

    //kicker down
    case 'h':
      Serial.write("kicker down\n");
      motorForward(3, 100);
      delay(300);
      motorStop(3);
      break;

    // kick
    case 'y':
      Serial.write("kick\n");
      motorBackward(3, 100);
      delay(300);
      motorForward(3, 100);
      delay(100);
      motorStop(3);
      break;
  }
}
