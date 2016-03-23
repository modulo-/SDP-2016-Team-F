#include "SDPArduino.h"
#include <Wire.h>
#include "comms.h"

// Motor mapping
#define MOTOR_LEFT 0 // polarity reversed
#define MOTOR_RIGHT 4 // polarity reversed
#define MOTOR_GRABBER 1 // forward is grab

#define ENCODER_LEFT 3
#define ENCODER_RIGHT 4
#define ENCODER_GRABBER 5

#define PIN_KICKER 6


//the radius of the left and right wheels
#define LR_WHEELBASE 0.15
//the radius of the middle wheel
#define MIDDLE_RADIUS 0.11
#define WHEEL_DIAM 0.082
#define PI 3.14159
#define WHEEL_CIRCUM WHEEL_DIAM*PI
//the number of steps in a rotation for the centre wheel
#define FULL_ROT_MIDDLE 25.0
#define FULL_ROT_LR 180.0


#define ACCEL_DETAILS

// CMD opcodes
#define CMD_MOVE 'm'
#define CMD_MOVEANDTURN 'M'
#define CMD_TURN 't'
#define CMD_KICK 'k'
#define CMD_DATA 'd'
#define CMD_GRAB 'g'
#define CMD_RELEASE 'r'
#define CMD_CELEBRATE 'c'
#define CMD_PING 'p'
#define CMD_TEST 'T'
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
    calibrateGrabbers();
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
            case CMD_GRAB: grab();
                           break;
            case CMD_RELEASE:
                           release();
                           break;
            case CMD_PING:
                           ping();
                           break;
            case CMD_TEST:
                           test();
                           break;
        }
    }
}

void parseOptions(byte* message) {
    char a = message[1];
    switch (a) {
        case 'm':
            degToMetre += message[2] - 127;
        default:
            break;
    }
}

void grab() {
    updateMotorPositions();
    //close flippers
    int time=millis();
    motorForward(MOTOR_GRABBER, 55);
    delay(800);
    if(positions[ENCODER_GRABBER]<=8){
        commSend("BC");
    }
    else{
        commSend("NC");
    }
    motorAllStop();
}

void release() {
    //move flippers away
    motorBackward(MOTOR_GRABBER, 55);
    int time=millis();
    delay(800);
    motorAllStop();
}

void calibrateGrabbers() {
    //move flippers away
    motorBackward(MOTOR_GRABBER, 55);
    delay(800);
    positions[ENCODER_GRABBER]=0;
    motorForward(MOTOR_GRABBER, 55);
    delay(800);
    motorAllStop();
}

void ping(){
    updateMotorPositions();
    printMotorPositions();
}

void test(){
    debugSend("test");
    /*
       delay(1000);
       bool motorL=false;
       bool motorR=false;
       bool motorG=false;

       motorForward(MOTOR_GRABBER,75);
       delay(800); 
       updateMotorPositions();
       motorG=positions[ENCODER_GRABBER]!=0;
       delay(800);
       motorBackward(MOTOR_GRABBER,75);
       updateMotorPositions();
       motorG=positions[ENCODER_GRABBER]!=0 | motorG;
       if(motorG){debugSend("grabbers OK");}
       else{ debugSend("some problem with grabbers");}

       motorAllStop();

       motorForward(MOTOR_LEFT,100);
       delay(800); 
       updateMotorPositions();
       motorL=positions[ENCODER_LEFT]!=0;
       delay(800);
       motorBackward(MOTOR_LEFT,100);
       updateMotorPositions();
       motorL=positions[ENCODER_LEFT]!=0 | motorL;
       if(motorL){debugSend("left OK");}
       else{ debugSend("some problem with left");}

       motorAllStop();

       motorForward(MOTOR_RIGHT,100);
       delay(800); 
       updateMotorPositions();
       motorR=positions[ENCODER_RIGHT]!=0;
       delay(800);
       motorBackward(MOTOR_RIGHT,100);
       updateMotorPositions();
       motorR=positions[ENCODER_RIGHT]!=0 | motorR;
       if(motorR){debugSend("right OK");}
       else{ debugSend("some problem with right");}

       motorAllStop();
     */
}

void doMove(byte * message) {
    int distance = (message[1] << 8) | message[2];
    move(distance);
}

void doMoveAndTurn(byte * message) {
    int firstTurn = (message[1] << 8) | message[2];
    int distance = (message[3] << 8) | message[4];
    int secondTurn = (message[5] << 8) | message[6]; // relative finish heading
    turn(firstTurn); // turn to calculated final abs heading
    move(distance); // move in relative heading
    delay(1000);
    turn(secondTurn); // turn to calculated final abs heading
}

void doTurn(byte * message) {
    int angle = (message[1] << 8) | message[2];
    turn(angle); // turn to calculated final abs heading
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



// move some distance in mm
void move(int distance) {
    if (Serial.available()) {
        return;
    };
    String numStr=String(distance);
    char numArr[numStr.length()+1];
    for(int k=0;k<numStr.length();k++){
        numArr[k]=numStr.charAt(k);
    }
    numArr[numStr.length()+1]='\0';
    debugSend(numArr);
    delay(100);
#ifdef ACCEL_DETAILS
    long maxDataPoints=10;
    int numberDataPoints=0;
    int dimensions=10;
    long history[dimensions][maxDataPoints];
    long prevLogTime=millis();
#endif
    //long degs = (distance * degToMetre) / 1000;
    long degs=(long) distance;

    bool forwards = (distance >= 0);
    //everything is confusing because the motors are mounted backwards
    resetMotorPositions();
    int start[] = {positions[0], positions[1]};

    int deltaL = start[0];
    int prevDeltaL =start[0];
    int deltaR = start[1];
    int prevDeltaR=start[1];
    bool finished=false;
    int leftPower = 150;
    int rightPower = 150;

    int lastChange=millis();

    long prevTime=millis();
    long time=millis();
    long deltaTime=0;
    
    long currentSpeed=0;
    long double targetSpeed=0;
    long maxSpeed=250;
    long double targetDelta=0;
    long acceleration=200;
    long deceleration=200;
    double pidFactor=1;

    while (!finished ){// && !Serial.available()) {
        time=millis();
        updateMotorPositions();
        deltaL = -(positions[ENCODER_LEFT] - start[0]);
        deltaR = -(positions[ENCODER_RIGHT] - start[1]);
        if (!forwards) {
            deltaL = -deltaL;
            deltaR = -deltaR;
        }
        //wait until they get started before doing anything
        if(time-prevTime>=10&&deltaL!=0&&deltaR!=0){
            deltaTime=time-prevTime;
            targetDelta+=(deltaTime*targetSpeed)/1000;
            currentSpeed=((1000*((deltaL-prevDeltaL)+(deltaR-prevDeltaR)))/2)/(deltaTime);
            double distanceToDecelerate=(currentSpeed*currentSpeed)/(2*deceleration);
            //accelerating
            if(currentSpeed<maxSpeed && abs(degs)-((deltaL+deltaR)/2)>(distanceToDecelerate)){
                targetSpeed=currentSpeed+ ((acceleration*(deltaTime))/1000.0);
                if(targetSpeed>maxSpeed)targetSpeed=maxSpeed;
                if(targetSpeed<0)targetSpeed=0;
            }
            else if(abs(degs)-((deltaL+deltaR)/2)<(distanceToDecelerate)){
                targetSpeed=currentSpeed - (deceleration*(deltaTime))/1000.0;
                if(targetSpeed<0)targetSpeed=0;
                if(targetSpeed>maxSpeed)targetSpeed=maxSpeed;
            }
            //logging
#ifdef ACCEL_DETAILS
            if(numberDataPoints<maxDataPoints&&time-prevLogTime>100){
                history[0][numberDataPoints]=deltaL; 
                history[1][numberDataPoints]=deltaR; 
                history[2][numberDataPoints]=micros(); 
                history[3][numberDataPoints]=leftPower;
                history[4][numberDataPoints]=rightPower; 
                history[5][numberDataPoints]=currentSpeed; 
                history[6][numberDataPoints]=targetSpeed; 
                history[7][numberDataPoints]=distanceToDecelerate; 
                history[8][numberDataPoints]=targetDelta; 
                history[9][numberDataPoints]=abs(degs)-((deltaL+deltaR)/2); 
                numberDataPoints++;
                prevLogTime=time;
            }
#endif
            leftPower += (targetDelta-deltaL)*pidFactor;
            if(leftPower>255)leftPower=255;
            else if(leftPower<0)leftPower=0;
            rightPower += (targetDelta-deltaR)*pidFactor;
            if(rightPower>255)rightPower=255;
            else if(rightPower<0)rightPower=0;
            prevDeltaL=deltaL;
            prevDeltaR=deltaR;
            prevTime=time;
        }
        //if we have reached our target and aren't moving, quit
        if(millis()-lastChange>2000){
            break;
        }
        if(deltaL-prevDeltaL>0||deltaR-prevDeltaR>0){
            lastChange=millis();
        }
        /*
        if((deltaL+deltaR)/2>abs(degs)){
            leftPower=0;
            rightPower=0;
        }
        */

        if (forwards) {
            motorBackward(MOTOR_LEFT, leftPower);
            motorBackward(MOTOR_RIGHT, rightPower);
        }
        else {
            motorForward(MOTOR_LEFT, leftPower);
            motorForward(MOTOR_RIGHT, rightPower);
        }
    }
    debugSend("finished");
    motorAllStop();
    updateMotorPositions();
    printMotorPositions();
#ifdef ACCEL_DETAILS
    for(int i=0;i<numberDataPoints;i++){
        for(int j=0;j<dimensions;j++){
            String numStr=String(history[j][i]);
            char numArr[numStr.length()+1];
            for(int k=0;k<numStr.length();k++){
                numArr[k]=numStr.charAt(k);
            }
            numArr[numStr.length()+1]='\0';
            debugSend(numArr);
            delay(10);
            if(j!=dimensions-1){
                debugSend(", ");
                delay(10);
            }
        }
        debugSend("\n");
        delay(10);
    }
#endif
    }

    //in a full rotation left should go 550 right should go 550
    // turn a certain number of degs
    void turn(long degs) {
        /*
           if (Serial.available()) {
           return;
           };
        //causes problems, don't know why
        //debugPrint("turning");
        //debugPrint((char*) (Serial.available() ? "true":"false"));
        bool cw = degs > 0;
        bool finished = false;
        //degrees of rotation of the wheel to rotation of the robot
        //because 1 reported 2 degree is 2 degrees of rotation
        float degToDegLR = (360 / FULL_ROT_LR) * ((WHEEL_CIRCUM) / (LR_WHEELBASE * PI));
        float degToDegC =  (360 / FULL_ROT_MIDDLE) * ((WHEEL_CIRCUM) / (2 * MIDDLE_RADIUS * PI));
        //target speed in deg/s
        int targetRotV = 70;
        //everything is confusing because the motors are mounted backwards
        degs = abs(degs);
        if (degs > 360)degs = degs % 360;
        updateMotorPositions();
        resetMotorPositions();
        updateMotorPositions();
        int start[] = {positions[ENCODER_LEFT], positions[ENCODER_RIGHT]};
        int prevPositions[] = {0, 0, 0};
        float deltaL = 0;
        float deltaR = 0;
        int pows[] = {40, 40};
        int startTime = millis();
        int prevTime = 0;
        while (!finished && !Serial.available()) {
        updateMotorPositions();
        deltaL = -(positions[ENCODER_LEFT] - start[0]) * degToDegLR;
        deltaR =  (positions[ENCODER_RIGHT] - start[1]) * degToDegLR;
        int deltas[] = {deltaL, deltaR};
        if (!cw) {
        deltaL = -deltaL;
        deltaR = -deltaR;
        }
        int time = millis();
        if ((deltaL + deltaR) / 2 >= degs) {
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
        }
        if (cw) {
        motorBackward(MOTOR_LEFT, pows[0]);
        motorForward(MOTOR_RIGHT, pows[1]);
        }
        else {
        motorForward(MOTOR_LEFT, pows[0]);
        motorBackward(MOTOR_RIGHT, pows[1]);
        }
        }
        motorAllStop();
        updateMotorPositions();
         */
    }

    void kick(int distance) { // distance in cm
        /*
           distance=abs(distance);
           int kickerTime = 0;
           int kickerStrength = 100;
           switch (distance) {
           default:
           kickerTime = 6+ ((distance-50)/50);
           break;
           }
           release();
           digitalWrite(PIN_KICKER, HIGH);

           delay(kickerTime);
        //put kicker back down
        digitalWrite(PIN_KICKER, LOW);
        //put grabbers back in
        delay(200);
        grab();
         */
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

    void updateMotorPositions() {
        // Request motor position deltas from rotary slave board
        Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);
        int8_t values[ROTARY_COUNT];
        //check for error
        bool error=true;
        bool changed=false;
        for (int i = 0; i < ROTARY_COUNT; i++) {
            int8_t a =(int8_t) Wire.read();  // Must cast to signed 8-bit type
            values[i]=a;
            if(a!=-1){error=false;}
            if(a!=0){changed=true;}
        }
        if(error){
            debugSend("cannot communicate with encoder board");
        }
        else{
            // Update the recorded motor positions
            for (int i = 0; i < ROTARY_COUNT; i++) {
                positions[i] += values[i]; 
            }
        }
    }

    void resetMotorPositions() {
        updateMotorPositions();
        for (int i = 0; i < ROTARY_COUNT; i++) {
            //the grabber needs an absolute position
            if(i!=ENCODER_GRABBER){
                positions[i] = 0;
            }
        }
    }

    void printMotorPositions() {
        debugSend("Motor positions: ");
        String positionStr;
        for(int i=0; i<ROTARY_COUNT;i++){
            positionStr.concat(positions[i]);
            if(i!=ROTARY_COUNT-1){
                positionStr.concat(',');
            }
        }
        char positionString[positionStr.length()+1];
        for(int i=0;i<positionStr.length();i++){
            positionString[i]=positionStr.charAt(i);
        }
        positionString[positionStr.length()+1]='\0';
        debugSend(positionString);
    }

    void commSend(char* str) {
        comms::send(str, 'c', strlen(str));
    }

    void debugSend(char* str) { 
        comms::send(str, 'd', strlen(str));
    }
