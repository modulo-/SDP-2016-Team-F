#include "SDPArduino.h"
#include <Wire.h>
#include "comms.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
//#include <avr/wdt.h>


// Motor mapping
#define MOTOR_LEFT 0 // polarity reversed
#define MOTOR_RIGHT 1 // polarity reversed
#define MOTOR_MIDDLE 2
#define MOTOR_KICKER 3
#define MOTOR_GRABBER 5 // no idea why!

// CMD opcodes
#define CMD_MOVE 'm'
#define CMD_MOVEANDTURN 'M'
#define CMD_TURN 't'
#define CMD_KICK 'k'
#define CMD_DATA 'd'

#define MAX_DATA_SIZE 255
#define SERIAL_DEBUG 1
#define DEVICE_ID '2'

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
  //indicator led for comms system, on indicates error
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  if (!comms::init("67", "~~~")) {
    digitalWrite(13, HIGH);
  }

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

  void process(void *data, size_t len) {
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
      case CMD_KICK:
        doKick(message);
        break;
      case CMD_DATA:
        doData(message);
        break;
    }
    free(data);
  }
}

void doMove(byte * message) {
  int distance = (message[1] << 8) | message[2];
  if(SERIAL_DEBUG) {
  Serial.println("doMove:");
  Serial.println(distance);
  }
  move(distance);
}

void doMoveAndTurn(byte * message) {
  int direction = (message[1] << 8) | message[2];
  int distance = (message[3] << 8) | message[4];
  int finalHeading = (message[5] << 8) | message[6]; // relative finish heading

  if(SERIAL_DEBUG) {
  Serial.println("doMoveAndTurn:");
  Serial.println(distance);
  Serial.println(direction);
  Serial.println(finalHeading);
  }
  
  int startHeading = getCurrentHeading(); // absolute start heading
  finalHeading = (startHeading + finalHeading + 360) % 360; // absulute finish heading

  turn(getHeadingDiff(direction, startHeading)); // turn to calculated final abs heading
  move(distance); // move in relative heading
  turn(getHeadingDiff(finalHeading, getCurrentHeading())); // turn to calculated final abs heading
}

void doTurn(byte * message) {
  int heading = (message[1] << 8) | message[2];

  int finalHeading = (getCurrentHeading() + heading + 360) % 360; // absolute finish heading

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
// distance in mm, direction in degrees

// What happens when the wheen counters overflow. Possibly reset them before each action?
void move(int distance) {
  long distanceCovered = 0;
  int startHeading = getCurrentHeading();
  long degToMetre = 1250;
  long degrees = (distance*degToMetre)/1000;
  
  if(SERIAL_DEBUG) {
  Serial.print("I am going to move:");
  Serial.println(degrees);
  }
  
  bool finished = false;
  bool forwards = (distance >= 0);
  //everything is confusing because the motors are mounted backwards
  if(forwards)degrees=-degrees;
  updateMotorPositions();
  int start[]={positions[0],positions[1]}; 
  
  if(SERIAL_DEBUG) {
  Serial.println("forwards");
  }
  int delta0=0;
  int delta1=0;
  int leftPower=100;
  int rightPower=100;
  int prevTime=millis();
  bool timeout;
  
  if(SERIAL_DEBUG) {
  Serial.print("Starting at: ");
  Serial.print(start[0]);
  Serial.print(" ");
  Serial.print(start[1]);
  Serial.print(" Going to:");
  Serial.print(positions[0]+degrees);
  Serial.print(" ");
  Serial.println(positions[1]+degrees);
}

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
      
  if(SERIAL_DEBUG) Serial.print("Left finished\r\n");
      leftPower=0;
    }
    if(rightPower!=0&& abs(delta1)>=abs(degrees)){
      
  if(SERIAL_DEBUG) Serial.print("Right finished\r\n");
      rightPower=0;
    }
    if((abs(delta0)>=abs(degrees)&&abs(delta1)>=abs(degrees)) || (leftPower==0 && rightPower==0)){
      finished = true;
      break;
    }
    else if(delta0-delta1>5){
      if(timeout){
        
  if(SERIAL_DEBUG) {
        Serial.print("Too far right, reducing power to left engine\r\n");
        Serial.print(" Left has gone:");
        Serial.print(delta0);
        Serial.print(" Right has gone:");
        Serial.print(delta1);
        Serial.print("\r\n");
  }
      }
      if(leftPower>90)leftPower=90;
    }
    else if(delta1-delta0>5){
      if(timeout){
        
  if(SERIAL_DEBUG) {
        Serial.print("Too far left, reducing power to right engine\r\n");
        Serial.print("Left has gone:");
        Serial.print(delta0);
        Serial.print(" Right has gone:");
        Serial.print(delta1);
        Serial.print("\r\n");
      }
      }
      if(rightPower>90)rightPower=90;
    }
    else{
      if(timeout) if(SERIAL_DEBUG) Serial.print("Going straight\r\n");
      if(leftPower!=0)leftPower=100;
      if(rightPower!=0)rightPower=100;
    }
    if(forwards){
      motorBackward(MOTOR_LEFT,leftPower);
      motorBackward(MOTOR_RIGHT,rightPower);
    }
    else{
      motorForward(MOTOR_LEFT,leftPower);
      motorForward(MOTOR_RIGHT,rightPower);
    }
  }
  motorAllStop();
  updateMotorPositions();
  
  if(SERIAL_DEBUG) {
  Serial.print("Finished at: ");
  Serial.print(positions[0]);
  Serial.print(" ");
  Serial.print(positions[1]);
  Serial.print("\r\n");
  }
}

int fullRot=300;
//in a full rotation left should go 550 right should go 550
// turn a certain number of degrees
void turn(long degrees){
  int targetHeading=((getCurrentHeading()+360)+degrees)%360;
  int startHeading=getCurrentHeading();
  
  if(SERIAL_DEBUG) {
  Serial.print("targetHeading:");
  Serial.println(targetHeading);
  Serial.print("currentHeading:");
  Serial.println(getCurrentHeading());
  Serial.print("diff:");
  Serial.println(getHeadingDiff(targetHeading,getCurrentHeading()));
  }
  if(degrees>360)degrees = degrees %360;
  if(degrees<-360)degrees = -((-degrees) %360);
  degrees=(degrees*fullRot)/360;
  bool cw=degrees>0;
	bool finished = false;
	//everything is confusing because the motors are mounted backwards
	degrees=-degrees;
	updateMotorPositions();
	int start[]={positions[0],positions[1]}; 
 
  if(SERIAL_DEBUG) {
	Serial.write(cw?"clockwise":"anticlockwise");
	Serial.println(degrees);
  }
	int delta0=0;
	int delta1=0;
	int leftPower=100;
	int rightPower=100;
	int prevTime=millis();
	bool timeout;
	while(!finished){
		updateMotorPositions();
		//timeout=false;
		timeout = (millis()-prevTime) > 1000;
		if(timeout){	
			prevTime=millis();
			printMotorPositions();
		}
		delta0=-(positions[0]-start[0]);
		delta1=(positions[1]-start[1]);
		if(!cw){
			delta0=-delta0;
			delta1=-delta1;
		}
		if(leftPower!=0&& abs(delta0)>=abs(degrees)){
			
  if(SERIAL_DEBUG) Serial.print("Left finished\r\n");
			leftPower=0;
		}
		if(rightPower!=0&& abs(delta1)>=abs(degrees)){
			
  if(SERIAL_DEBUG) Serial.print("Right finished\r\n");
			rightPower=0;
		}
		if((abs(delta0)>=abs(degrees)&&abs(delta1)>=abs(degrees)) || (leftPower==0 && rightPower==0)){
			finished = true;
			break;
		}
		else if(delta0-delta1>5){
			if(timeout){
        
  if(SERIAL_DEBUG) {
				Serial.print("Too far right, reducing power to left engine\r\n");
				Serial.print(" Left has gone:");
				Serial.print(delta0);
				Serial.print(" Right has gone:");
				Serial.print(delta1);
				Serial.print("\r\n");
  }
			}
			if(leftPower>80)leftPower=80;
		}
		else if(delta1-delta0>5){
			if(timeout){
        
  if(SERIAL_DEBUG) {
				Serial.print("Too far left, reducing power to right engine\r\n");
				Serial.print("Left has gone:");
				Serial.print(delta0);
				Serial.print(" Right has gone:");
				Serial.print(delta1);
				Serial.print("\r\n");
  }
			}
			if(rightPower>80)rightPower=80;
		}
		else{
			if(timeout)  if(SERIAL_DEBUG) Serial.print("Going straight\r\n");
			if(leftPower!=0)leftPower=90;
			if(rightPower!=0)rightPower=90;
		}
		if(cw){
			motorBackward(MOTOR_LEFT,leftPower);
			motorForward(MOTOR_RIGHT,rightPower);
		}
		else{
			motorForward(MOTOR_LEFT,leftPower);
			motorBackward(MOTOR_RIGHT,rightPower);
		}
	}
	motorAllStop();
	updateMotorPositions();
  printMotorPositions();
  delay(1000);
  if(abs(getHeadingDiff(targetHeading,getCurrentHeading()))>30){
    
  if(SERIAL_DEBUG)     Serial.println("too far out...correcting");
    turn(getHeadingDiff(targetHeading,getCurrentHeading()));
  }
  
  if(SERIAL_DEBUG) {
	Serial.print("Finished at: ");
  Serial.print("currentHeading:");
  Serial.println(getCurrentHeading());
	Serial.print(positions[0]);
	Serial.print(" ");
	Serial.print(positions[1]);
	Serial.print("\r\n");
}
}

void kick(int distance) { // distance in cm
  int kickerTime=0;
  int kickerStrength=100;
  switch(distance){
    case 50:
      kickerTime=130;
      break;
    case 100:
      kickerTime=150;
      break;
    case 150:
      kickerTime=173;
      break;
    default:
      kickerTime=120+ (int) distance*0.3533;
      break;
  }
  //close flippers
  motorBackward(MOTOR_GRABBER, 100);
  //move kicker back out way
  motorForward(MOTOR_KICKER, 30);
  delay(800);
  //
  motorForward(MOTOR_KICKER, 0);
  motorForward(MOTOR_GRABBER, 80);
  delay(800);
  motorAllStop();
  
  if(SERIAL_DEBUG) {
  Serial.write("kicking at P:");
  Serial.print(kickerStrength);
  Serial.write(" T:");
  Serial.print(kickerTime);
  Serial.write("\r\n");
}
  motorBackward(MOTOR_KICKER, kickerStrength);
  delay(kickerTime);
  motorForward(MOTOR_KICKER, 30);
  delay(1000);
  motorStop(MOTOR_KICKER);
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
    return 360 - ((int) orientation.heading);
  }
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
}
