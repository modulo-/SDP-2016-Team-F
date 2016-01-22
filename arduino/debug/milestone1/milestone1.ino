#include "SDPArduino.h"
#include <Wire.h>
#include "comms.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

#define ROTARY_SLAVE_ADDRESS 5

// Motor mapping
#define MOTOR_LEFT 0 // polarity reversed
#define MOTOR_RIGHT 1 // polarity reversed
#define MOTOR_MIDDLE 2
#define MOTOR_KICKER 3
#define MOTOR_GRABBER 4

Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);

int targetHeading;

namespace comms {
    const char DEVICEID = '2';

    void process(void *data, size_t len) {
        free(data);
    }
}


#define CMD_MOVE 'm'
#define CMD_KICK 'k'
#define CMD_DATA 'd'
#define MAX_DATA_SIZE 255
#define DEVICEID 2

byte * dataBytes = (byte *) malloc(MAX_DATA_SIZE * sizeof(byte));
int dataFreq = 2;
int dataLen = 100;

void setup() {
  SDPsetup();
  helloWorld();

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  if (!comms::init("67", "~~~", &process)) {
    digitalWrite(13, HIGH);
  }

  for(int i = 0; i<MAX_DATA_SIZE; i++) {
    dataBytes[i] = (byte) i; // fixme
  }
}

void loop() {
  comms::poll();

  delay(100);
}

// called by the comms lib
void process(void *data, size_t len){
  
  // message format: [1B opcode][2B arg1][2B arg2]
  //  message = "m\000\020\000\050\000"; // move 20cm in relative heading 50 deg to right
  //  message = "t\255\206\000"; // turn 50 deg to the left (-50 is same as 206)
  //  message = "k\000\050\000"; // kick the ball to 50cm
//  message = "d\000\010\000\001ABCDEFGHIJ\0"; // send 100 bytes of data to i2c at 25hz followed by the data
  byte * message = (byte *) data;

//  for(int i=0; i<10; i++) {
//    
//    Wire.beginTransmission(0x45);
//    Wire.write(message[i]);
//    Wire.endTransmission();
//    delay(100);
//  }

  switch (message[0]) {
    case 'm':
      doMove(message);
      break;
    case 't':
      doTurn(message);
      break;
    case 'k':
      doKick(message);
      break;
    case 'd':
      doData(message);
      break;
  }
}

void doMove(byte * message) {
  int distance = (message[1] << 8) | message[2];
  int direction = (message[3] << 8) | message[4];
  int finalHeading = (message[5] << 8) | message[6]; // relative finish heading

  int startHeading = getCurrentHeading(); // absolute start heading
  finalHeading = (startHeading + finalHeading + 360) % 360; // absulute finish heading

  move(direction, distance); // move in relative heading
  turn(getHeadingDiff(finalHeading, getCurrentHeading())); // turn to calculated final abs heading
}

void doTurn(byte * message) {
  int heading = (message[1] << 8) | message[2];

  int finalHeading = (message[5] << 8) | message[6]; // relative finish heading
  finalHeading = (getCurrentHeading() + finalHeading + 360) % 360; // absulute finish heading

  turn(getHeadingDiff(finalHeading, getCurrentHeading())); // turn to calculated final abs heading
}

void doKick(byte * message) {
  int distance = (message[1] << 8) | message[2];
  kick(distance);
}

void doData(byte * message) {
  int part =  message[1];
  if (part == 0) {
    dataFreq = message[2];
    dataLen = message[3];
  } else {
    int firstByte = message[2];
    int chunkLen = message[3];
    byte * chunk = &message[4];
    memcpy(&dataBytes[firstByte], chunk, chunkLen);
    if (firstByte+chunkLen >= dataLen) {
      sendData();
    }
  }

}

void sendData() {
  for (int i = 0; i < dataLen; i++) {
    byte data = dataBytes[i]; // get message from queue
    Wire.beginTransmission(0x45);
    Wire.write(data);
    Wire.endTransmission();
    delay(1000 / dataFreq);
  }
}
// move some distance in specified direction, idealy by changing heading minimally
void move(int direction, int distance) {

}

// turn on the spot
void turn(int dir) { // positive - right, negative - left
  if (dir > 0) { // turn right
    motorBackward(0, 100);
    motorForward(1, 100);
    delay(abs(dir)); //fixme
  } else { // turn left
    motorForward(0, 100);
    motorBackward(1, 100);
    delay(abs(dir)); //fixme
  }
}

void kick(int distance) { // distance in cm

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



int getBatteryVoltage() {
  int relativeVoltage = analogRead(2); // Input on A2.
  float absoluteVoltage = (5.0 * relativeVoltage) / 1023.0;
  /*
    Need to multiply by 2*10 since the potential divider on the board
    divided the voltage by 2, and we want a larger precision number.
    I.e. voltage will go from 0-100.
  */
  int responseVoltage = (int) absoluteVoltage * 20;
  return responseVoltage;
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

