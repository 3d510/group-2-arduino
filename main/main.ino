#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

#define TEST_TIME 0.002
#define FRONT_RIGHT A0
#define FRONT_CENTER A1
#define FRONT_LEFT A2
#define LEFT A3
#define RIGHT A4
#define BACK A5 
#define READ_TIMES 7
#define encoderRPinA 3
#define encoderRPinB 5
#define encoderLPinA 11
#define encoderLPinB 13

volatile int encoderRPos = 0;
int aencoderRPinALast = LOW;

volatile int encoderLPos = 0;
int encoderLPinALast = LOW;

int n = LOW;
//int counter = 0;
DualVNH5019MotorShield md;
long long Time;

void setup() {
  pinMode (encoderLPinA, INPUT);
  pinMode (encoderLPinB, INPUT);
  pinMode (encoderRPinA, INPUT);
  pinMode (encoderRPinB, INPUT);
  
  Serial.begin (9600);
  // attachInterrupt(1, doEncoder, RISING); 
  PCintPort::attachInterrupt(encoderLPinA, &doEncoderLeft, RISING);
  PCintPort::attachInterrupt(encoderRPinA, &doEncoderRight, RISING);
  md.init();
}

bool done = false;
int count = 1;
float rpms[200];

void loop() {
  if (!done) {    
    //md.setSpeeds(100, 0);
    
    //delay(3000);
    //Serial.print(readRpmWithInterrupt(false, 0.02));
//    rpms[0] = abs(readRpmWithInterrupt(false, 0.02));
//    md.setSpeeds(300, 0);
//    while (count < 200) {
//      rpms[count] = abs(readRpmWithInterrupt(false, 0.02));
//      count++;
//    }
//    for (int i = 0; i < count; i++) {
//      Serial.print(i);
//      Serial.print(","); 
//      Serial.println(rpms[i]);
//    }
//    done = true;
  } else {
    md.setSpeeds(0,0);
  }

}

float readRpmWithInterrupt(bool isLeftWheel, float delayTime) { // delayTime in seconds
  if (isLeftWheel) {
    encoderLPos = 0;
    delay(delayTime * 1000);
    float rpm = 60 / (562.25 / ((encoderLPos - 1) / delayTime));
    return rpm;
  } else {
    encoderRPos = 0;
    delay(delayTime * 1000);
    float rpm = 60 / (562.25 / ((encoderRPos - 1) / delayTime));
    return rpm;
  }
}

void doEncoderLeft() {
//  if (digitalRead(encoderLPinA) == digitalRead(encoderLPinB)) {
//    encoderLPos++;
//  } else {
//    encoderLPos--;
//  }
  encoderLPos++;
}

void doEncoderRight() {
//  if (digitalRead(encoderRPinA) == digitalRead(encoderRPinB)) {
//    encoderRPos++;
//  } else {
//    encoderRPos--;
//  }
  encoderRPos++;
}

float readSingleSensor(int sensorNumber) {
  // read the pin 7 times to get median value
  int sensorVal[READ_TIMES];
  int sortedSensorVal[READ_TIMES];
  for (int i = 0; i < READ_TIMES; i++) {
    sensorVal[i] = analogRead(sensorNumber);         
    delay(10);
  }
  sortedSensorVal[0] = sensorVal[0];
  for (int i = 1; i < READ_TIMES; i++) {
    int id = 0;
    while (id < i && sortedSensorVal[id] <= sensorVal[i])
      id++;
    if (id == i) {
      sortedSensorVal[i] = sensorVal[i];
    } else {
      for (int j = i - 1; j >= id; j--) {
        sortedSensorVal[j+1] = sensorVal[j]; 
      } 
      sortedSensorVal[id] = sensorVal[i]; 
    }
  }
  int readValue = sortedSensorVal[READ_TIMES/2];

  // deduce distance from value read from analog pin
  switch (sensorNumber) {
    case FRONT_RIGHT: 
      return 8544.11/readValue - 7.29;
    case FRONT_LEFT: 
      return 8068.98/readValue - 5.63;
    case LEFT:
      return 8766.22/readValue - 7.35;
    case RIGHT:
      return 7904.38/readValue - 5.62;
    case BACK:
      return 8068.41/readValue - 6.77;
  }
  
}

float readRpmWithoutInterrupt(bool isLeftWheel) {
//  Time = micros();
//  md.setSpeeds(400,400);
//  if (Time <= 2000000) {
//    n = digitalRead(encoderRPinA);
//    if ((encoderRPinALast == LOW) && (n == HIGH)) {
//      if (digitalRead(encoderRPinB) == LOW) {
//        encoderRPos--;
//      } else {
//        encoderRPos++;
//      }
//    }
//    encoderRPinALast = n;
//  } else {
//    if (counter == 0)
//      Serial.print(encoderRPos);
//    counter = 1;
//  }
}


