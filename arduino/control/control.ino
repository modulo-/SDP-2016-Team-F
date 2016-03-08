#include "SDPArduino.h"
#include <Wire.h>
#include "comms.h"
//#define COMPASS
#ifdef COMPASS
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <avr/wdt.h>
#endif


// Motor mapping
#define MOTOR_LEFT 3 // polarity reversed
#define MOTOR_RIGHT 0 // polarity reversed
#define MOTOR_MIDDLE 0 // forward is anticlockwise
#define MOTOR_KICKER 5 // forward is kick
#define MOTOR_GRABBER 2 // forward is grab

#define ENCODER_LEFT 5
#define ENCODER_RIGHT 1
#define ENCODER_MIDDLE 5 //+ve is clockwise
#define ENCODER_GRABBER 4
    

#define PIN_KICKER 6

//the radius of the left and right wheels
#define LR_WHEELBASE 0.15
//the radius of the middle wheel
#define MIDDLE_RADIUS 0.11
#define WHEEL_DIAM 0.05
#define PI 3.14159
#define WHEEL_CIRCUM WHEEL_DIAM*PI
//the number of steps in a rotation for the centre wheel
#define FULL_ROT_MIDDLE 25.0
#define FULL_ROT_LR 180.0
//#define SERIAL_DEBUG

// CMD opcodes
#define CMD_MOVE 'm'
#define CMD_MOVEANDTURN 'M'
#define CMD_TURN 't'
#define CMD_KICK 'k'
#define CMD_DATA 'd'
#define CMD_GRAB 'g'
#define CMD_RELEASE 'r'
#define OPTIONS 'o'

#define MAX_DATA_SIZE 255
#define DEVICE_ID '2'
#define TURN_ALLOWANCE 3

//motor positioning
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define PRINT_DELAY 200

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};
bool turnCorrection = false;
int kickerTime50 = 80;
int kickerTime100 = 110;
int kickerTime150 = 135;
long degToMetre = 1250;
#ifdef COMPASS
Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);
#endif

int targetHeading;

void doMoveAndTurn(byte * message);
void doTurn(byte * message);
void doKick(byte * message);
void doData(byte * message);


byte * dataBytes = (byte *) malloc(MAX_DATA_SIZE * sizeof(byte));
int dataFreq = 2;
int dataLen = 100;

struct goal {
  int distance;
  int heading;
};

void setup() {
  SDPsetup();
  Serial.flush();
  while (Serial.available()) {
    Serial.read();
  }

  // indicator led for comms system, on indicates error
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  if (!comms::init("67", "~~~")) {
    digitalWrite(13, HIGH);
  }
  pinMode(PIN_KICKER, OUTPUT);

  Wire.begin(ROTARY_SLAVE_ADDRESS);  // I2C slave at given address
  initSensors();

  //Initialise the WDT and flush the serial...
  //wdt_enable(WDTO_1S);
  Serial.flush();
}

void loop() {
  comms::poll();
}

namespace comms {
const char DEVICEID = DEVICE_ID;

void process(void const *data, size_t len) {
  byte * message = (byte *) data;

  switch (message[0]) {
    case CMD_MOVE:
      doMove(message);
      break;
    case CMD_MOVEANDTURN:
      doMoveAndTurn(message);
      break;
    case CMD_TURN:
      doTurn(message);
      break;
    case OPTIONS:
      parseOptions(message);
      break;
    case CMD_KICK:
      doKick(message);
      break;
    case CMD_DATA:
      doData(message);
      break;
    case CMD_GRAB:
      grab();
      break;
    case CMD_RELEASE:
      release();
  }
  //free(data);
}
}

void parseOptions(byte* message) {
  char a = message[1];
  switch (a) {
    case 't':
      turnCorrection = message[2] >= 1;
      break;
    case 'k':
      switch (message[2]) {
        case 50:
          kickerTime50 += message[3] - 127;
          break;
        case 100:
          kickerTime100 += message[3] - 127;
          break;
        case 150:
          kickerTime150 += message[3] - 127;
          break;
      }
    case 'm':
      degToMetre += message[2] - 127;
    default:
      break;
  }
}

void grab() {
  //close flippers
  motorForward(MOTOR_GRABBER, 25);
  delay(800);
  motorAllStop();
}

void release() {
  //move flippers away
  motorBackward(MOTOR_GRABBER, 25);
  delay(800);
  motorAllStop();
}

void doMove(byte * message) {
  int distance = (message[1] << 8) | message[2];
#ifdef SERIAL_DEBUG
  Serial.println("doMove:");
  Serial.println(distance);
}
#endif
move(distance);
}

void doMoveAndTurn(byte * message) {
  int direction = (message[1] << 8) | message[2];
  int distance = (message[3] << 8) | message[4];
  int finalHeading = (message[5] << 8) | message[6]; // relative finish heading

#ifdef SERIAL_DEBUG
  Serial.println("doMoveAndTurn:");
  Serial.println(distance);
  Serial.println(direction);
  Serial.println(finalHeading);
#endif

  int startHeading = getCurrentHeading(); // absolute start heading
  finalHeading = (startHeading + finalHeading + 360) % 360; // absulute finish heading

#ifdef SERIAL_DEBUG
  Serial.println("Augmented headings:");
  Serial.println(startHeading);
  Serial.println(finalHeading);
#endif

  turn(direction, 0); // turn to calculated final abs heading
  move(distance); // move in relative heading
  delay(1000);
  turn(getHeadingDiff(finalHeading, getCurrentHeading()), 0); // turn to calculated final abs heading
}

void doTurn(byte * message) {
  int heading = (message[1] << 8) | message[2];

  int finalHeading = (getCurrentHeading() + heading + 360) % 360; // absolute finish heading

  turn(getHeadingDiff(finalHeading, getCurrentHeading()), 0); // turn to calculated final abs heading
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

    for (int i = 0; i < MAX_DATA_SIZE; i++) {
      dataBytes[i] = (byte) i; // fixme
    }
  } else if (part == 0xff) {
    sendData();
  } else {
    int firstByte = message[2];
    int chunkLen = message[3];
    byte * chunk = &message[4];
    memcpy(&dataBytes[firstByte], chunk, chunkLen);
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



// move some distance in specified direction, ideally by changing heading minimally
void move(int distance) {
  if (Serial.available()) {
    return;
  };
  resetMotorPositions();
  long distanceCovered = 0;
  int startHeading = getCurrentHeading();
  long degrees = (distance * degToMetre) / 1000;

#ifdef SERIAL_DEBUG
  Serial.print("I am going to move:");
  Serial.println(degrees);
#endif

  bool finished = false;
  bool forwards = (distance >= 0);
  //everything is confusing because the motors are mounted backwards
  if (forwards)degrees = -degrees;
  updateMotorPositions();
  int start[] = {positions[0], positions[1]};

  int delta0 = 0;
  int delta1 = 0;
  int leftPower = 100;
  int rightPower = 100;
  int prevTime = millis();
  bool timeout;

#ifdef SERIAL_DEBUG
  Serial.print("Starting at: ");
  Serial.print(start[0]);
  Serial.print(" ");
  Serial.print(start[1]);
  Serial.print(" Going to:");
  Serial.print(positions[0] + degrees);
  Serial.print(" ");
  Serial.println(positions[1] + degrees);
#endif

  while (!finished && !Serial.available()) {
    updateMotorPositions();
    delta0 = -(positions[ENCODER_LEFT] - start[0]);
    delta1 = -(positions[ENCODER_RIGHT] - start[1]);
    if (!forwards) {
      delta0 = -delta0;
      delta1 = -delta1;
    }
    if (leftPower != 0 && abs(delta0) >= abs(degrees)) {
      leftPower = 0;
    }
    if (rightPower != 0 && abs(delta1) >= abs(degrees)) {
      rightPower = 0;
    }
    if ((abs(delta0) >= abs(degrees) && abs(delta1) >= abs(degrees)) || (leftPower == 0 && rightPower == 0)) {
      finished = true;
      break;
    }
    else if (delta0 - delta1 > 5) {
      if (leftPower > 90)leftPower = 90;
    }
    else if (delta1 - delta0 > 5) {
      if (rightPower > 90)rightPower = 90;
    }
    else {
      if (leftPower != 0)leftPower = 100;
      if (rightPower != 0)rightPower = 100;
    }
    if (forwards) {
      motorBackward(MOTOR_LEFT, leftPower);
      motorBackward(MOTOR_RIGHT, rightPower);
    }
    else {
      motorForward(MOTOR_LEFT, leftPower);
      motorForward(MOTOR_RIGHT, rightPower);
    }
  }
  motorAllStop();
  updateMotorPositions();

#ifdef SERIAL_DEBUG
  Serial.print("Finished at: ");
  Serial.print(positions[0]);
  Serial.print(" ");
  Serial.print(positions[1]);
  Serial.print("\r\n");
#endif
}

//in a full rotation left should go 550 right should go 550
// turn a certain number of degrees
void turn(long degrees, int depth) {
  if (Serial.available()) {
    return;
  };
  //causes problems, don't know why
  //debugPrint("turning");
  //debugPrint((char*) (Serial.available() ? "true":"false"));
  bool cw = degrees > 0;
  bool finished = false;
  //degrees of rotation of the wheel to rotation of the robot
  //because 1 reported 2 degree is 2 degrees of rotation
  float degToDegLR = (360 / FULL_ROT_LR) * ((WHEEL_CIRCUM) / (LR_WHEELBASE * PI));
  float degToDegC =  (360 / FULL_ROT_MIDDLE) * ((WHEEL_CIRCUM) / (2 * MIDDLE_RADIUS * PI));
#ifdef SERIAL_DEBUG
  Serial.print("degToDegLR:");
  Serial.print(degToDegLR);
  Serial.print(" degToDegC:");
  Serial.println(degToDegC);
#endif
  //target speed in deg/s
  int targetRotV = 70;
  //everything is confusing because the motors are mounted backwards
  degrees = abs(degrees);
  if (degrees > 360)degrees = degrees % 360;
  updateMotorPositions();
  resetMotorPositions();
  updateMotorPositions();
#ifdef SERIAL_DEBUG
  printMotorPositions();
#endif
  int start[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT], positions[ENCODER_MIDDLE]};
  int prevPositions[] = {0, 0, 0};
#ifdef SERIAL_DEBUG
  Serial.write(cw ? "clockwise " : "anticlockwise ");
  Serial.println(degrees);
#endif
  float deltaL = 0;
  float deltaR = 0;
  float deltaC = 0;
  int pows[] = {40, 40};
  int startTime = millis();
  int prevTime = 0;
#ifdef SERIAL_DEBUG
  int prevPrint = millis();
  bool timeout;
#endif
  while (!finished && !Serial.available()) {
#ifdef SERIAL_DEBUG
    timeout = false;
    if (millis() - prevPrint > 100) {
      timeout = true;
      prevPrint = millis();
    }
#endif
    updateMotorPositions();
    deltaL = -(positions[ENCODER_LEFT] - start[0]) * degToDegLR;
    deltaR =  (positions[ENCODER_RIGHT] - start[1]) * degToDegLR;
    deltaC = (positions[ENCODER_MIDDLE] - start[2]) * degToDegC;
    int deltas[] = {deltaL, deltaR, deltaC};
    if (!cw) {
      deltaL = -deltaL;
      deltaR = -deltaR;
      deltaC = -deltaC;
    }
    int time = millis();
    if ((deltaL + deltaR) / 2 >= degrees) {
      finished = true;
    }
    else if (time - prevTime > 500) {
      int i = 0;
      int diffs[] = {deltaL - prevPositions[0], deltaR - prevPositions[1]};
      for (i = 0; i < 2; i++) {
        float rotV = 1000 * ((float)diffs[i] / (time - prevTime));
        if (rotV > targetRotV) {
          pows[i] -= 5;
        }
        if (rotV < targetRotV) {
          pows[i] += 5;
        }
      }
      prevPositions[0] = deltaL;
      prevPositions[1] = deltaR;
      prevTime = time;
#ifdef SERIAL_DEBUG
      printMotorPositions();
#endif
    }
    if (cw) {
      motorBackward(MOTOR_LEFT, pows[0]);
      motorForward(MOTOR_RIGHT, pows[1]);
      //motorBackward(MOTOR_MIDDLE, powC);
    }
    else {
      motorForward(MOTOR_LEFT, pows[0]);
      motorBackward(MOTOR_RIGHT, pows[1]);
      //motorForward(MOTOR_MIDDLE, powC);
    }
  }
  motorAllStop();
  updateMotorPositions();
#ifdef SERIAL_DEBUG
  Serial.print("degrees:");
  Serial.print(degrees);
  Serial.print("\ndeltaL:");
  Serial.print(deltaL);
  Serial.print("\ndeltaR:");
  Serial.print(deltaR);
  Serial.print("\ndeltaC:");
  Serial.print(deltaC);
  Serial.print("\npowL:");
  Serial.print(pows[0]);
  Serial.print(" powR:");
  Serial.print(pows[1]);
  Serial.print("\ntargetRotV:");
  Serial.print(targetRotV);
  Serial.print("\n");
  printMotorPositions();
#endif
}

void kick(int distance) { // distance in cm
  int kickerTime = 0;
  int kickerStrength = 100;
  switch (distance) {
    case 50:
      kickerTime = kickerTime50;
      break;
    case 100:
      kickerTime = kickerTime100;
      break;
    case 150:
      kickerTime = kickerTime150;
      break;
    default:
      kickerTime = distance;
      break;
  }
  //close flippers
  //motorForward(MOTOR_GRABBER, 20);
  //delay(800);
  //move flippers away
  //motorBackward(MOTOR_GRABBER, 25);
  //delay(800);
  //motorAllStop();
  //kick
  release();
  digitalWrite(PIN_KICKER, HIGH);

  delay(kickerTime);
  //put kicker back down
  digitalWrite(PIN_KICKER, LOW);
  //put grabbers back in
  //motorForward(MOTOR_GRABBER, 25);
  delay(200);
  grab();
  //motorAllStop();
#ifdef SERIAL_DEBUG
  Serial.write("kicking at P:");
  Serial.print(kickerStrength);
  Serial.write(" T:");
  Serial.print(kickerTime);
  Serial.write("\r\n");
#endif
}


void initSensors()
{
#ifdef COMPASS
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
#endif
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
#ifdef COMPASS
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Y, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    return 360 - ((int) orientation.heading);
  }
#endif
#ifndef COMPASS
  return 0;
#endif
}



void updateMotorPositions() {
  // Request motor position deltas from rotary slave board
  Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);

  // Update the recorded motor positions
  for (int i = 0; i < ROTARY_COUNT; i++) {
    positions[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
  }
}

void resetMotorPositions() {
  for (int i = 0; i < ROTARY_COUNT; i++) {
    positions[i] = 0;
  }
}

void printMotorPositions() {
  Serial.print("Motor positions: ");
  for (int i = 0; i < ROTARY_COUNT; i++) {
    Serial.print(positions[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void debugPrint(char* str) {
  comms::send(str, 'c', strlen(str));
}
