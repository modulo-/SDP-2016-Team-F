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

//motor positioning
#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 2
#define PRINT_DELAY 200

// Initial motor position is 0.
int positions[ROTARY_COUNT] = {0};

Adafruit_9DOF                  dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro = Adafruit_L3GD20_Unified(20);

int targetHeading;

void doMove(byte * message);
void doTurn(byte * message);
void doKick(byte * message);
void doData(byte * message);

namespace comms {
  const char DEVICEID = '2';
  
  void process(void *data, size_t len) {
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
  //indicator led for comms system, on indicates error
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  if (!comms::init("67", "~~~")) {
    digitalWrite(13, HIGH);
  }

  for (int i = 0; i < MAX_DATA_SIZE; i++) {
    dataBytes[i] = (byte) i; // fixme
  }
  Wire.begin();
}

void loop() {
  comms::poll();

  delay(100);
}

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

void doMove(byte * message) {
  int distance = (message[1] << 8) | message[2];
  int direction = (message[3] << 8) | message[4];
  int finalHeading = (message[5] << 8) | message[6]; // relative finish heading

  int startHeading = getCurrentHeading(); // absolute start heading
  direction = (direction + finalHeading + 360) % 360; // absulute finish heading
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
// distance in metres, distance in degrees
void move(int direction, int distance) {
	turn(getHeadingDiff(getCurrentHeading(),direction));
	int degToMetre = 1250;
	int degrees = (distance/1000) *  degToMetre;
	bool finished = false;
	bool forwards = true;
	//everything is confusing because the motors are mounted backwards
	if(forwards)degrees=-degrees;
	updateMotorPositions();
	int start[]={positions[0],positions[1]}; 
	Serial.write("forwards\r\n");
	int delta0=0;
	int delta1=0;
	int leftPower=100;
	int rightPower=100;
	int prevTime=millis();
	bool timeout;
	Serial.print("Starting at: ");
	Serial.print(start[0]);
	Serial.print(" ");
	Serial.print(start[1]);
	Serial.print(" Going to:");
	Serial.print(positions[0]+degrees);
	Serial.print(" ");
	Serial.print(positions[1]+degrees);
	Serial.print("\r\n");
	while(!finished){
		updateMotorPositions();
		timeout=false;
		//timeout = (millis()-prevTime) > 1000;
		if(timeout){	
			prevTime=millis();
			printMotorPositions();
		}
		delta0=-(positions[0]-start[0]);
		delta1=-(positions[1]-start[1]);
		if(!forwards){
			delta0=-delta0;
			delta1=-delta1;
		}
		if(leftPower!=0&& abs(delta0)>=abs(degrees)){
			Serial.print("Left finished\r\n");
			leftPower=0;
		}
		if(rightPower!=0&& abs(delta1)>=abs(degrees)){
			Serial.print("Right finished\r\n");
			rightPower=0;
		}
		if((abs(delta0)>=abs(degrees)&&abs(delta1)>=abs(degrees)) || (leftPower==0 && rightPower==0)){
			finished = true;
			break;
		}
		else if(delta0-delta1>5){
			if(timeout){
				Serial.print("Too far right, reducing power to left engine\r\n");
				Serial.print(" Left has gone:");
				Serial.print(delta0);
				Serial.print(" Right has gone:");
				Serial.print(delta1);
				Serial.print("\r\n");
			}
			if(leftPower>90)leftPower=90;
		}
		else if(delta1-delta0>5){
			if(timeout){
				Serial.print("Too far left, reducing power to right engine\r\n");
				Serial.print("Left has gone:");
				Serial.print(delta0);
				Serial.print(" Right has gone:");
				Serial.print(delta1);
				Serial.print("\r\n");
			}
			if(rightPower>90)rightPower=90;
		}
		else{
			if(timeout)Serial.print("Going straight\r\n");
			if(leftPower!=0)leftPower=100;
			if(rightPower!=0)rightPower=100;
		}
		if(forwards){
			motorBackward(0,leftPower);
			motorBackward(1,rightPower);
		}
		else{
			motorForward(0,leftPower);
			motorForward(1,rightPower);
		}
	}
	motorAllStop();
	updateMotorPositions();
	Serial.print("Finished at: ");
	Serial.print(positions[0]);
	Serial.print(" ");
	Serial.print(positions[1]);
	Serial.print("\r\n");
}
//in a full rotation left should go 120 right should go 80
// turn a certain number of degrees
void turn(int dir) { // positive - right, negative - left
  int startHeading=getCurrentHeading();
  int finalHeading=startHeading + dir;
  if (dir > 0) { // turn right
    motorBackward(0, 100);
    motorForward(1, 100);
    motorBackward(2, 100);
    //delay(abs(1600.0*(dir/360.0)));
    while(abs(getCurrentHeading()-finalHeading)>10){

    }
  } else { // turn left
    motorForward(0, 100);
    motorBackward(1, 100);
    motorForward(2, 100);
    //delay(abs(1600.0*(dir/360.0)));
    while(abs(getCurrentHeading()-finalHeading)>10){

    }
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

