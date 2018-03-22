#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <string.h>

//with small battery, kp left = 13, kp right = 20
//with big battery, 17;1.5;0;20;2;0;26;26;10;100;250 can go 10 squares
#define L1 A1
#define F3 A0
#define R2 A2
#define R1 A3
#define F1 A4
#define BACK A5 

#define TEST_TIME 0.002
#define READ_TIMES 17
#define FORWARD 0
#define ROTATE_CW 1
#define ROTATE_CCW 2
#define BACKWARD 3

const int encoderRPinA = 11;
const int encoderRPinB = 13;
const int encoderLPinA = 3;
const int encoderLPinB = 5;
const double iteration_time = 0.01;
const double wheel_radius = 3; // cm
const double robot_radius = 9.5; // cm
const double distBetweenSensors = 13.5;
const double adjustEpsilon = 0.5;

volatile int encoderRPos = 0;
int encoderRPinALast = LOW;

volatile int encoderLPos = 0;
int encoderLPinALast = LOW;

int prevError = 0;

DualVNH5019MotorShield md;

int ticksDistance = 0;
double currentPulseTime1 = 0, prevPulseTime1 = 0;
double currentPulseTime2 = 0, prevPulseTime2 = 0;
double desired_rpm = 90;

void setup() {
  pinMode (encoderLPinA, INPUT);
  pinMode (encoderLPinB, INPUT);
  pinMode (encoderRPinA, INPUT);
  pinMode (encoderRPinB, INPUT);
  
  Serial.begin(9600);
  PCintPort::attachInterrupt(encoderLPinA, &doEncoderLeft, RISING);
  PCintPort::attachInterrupt(encoderRPinA, &doEncoderRight, RISING);
  md.init();
}

bool done = false;
int count = 1;
float rpms[200];
int leftTickCount[100];
int rightTickCount[100];
int tickDiff[100];

void loop() {
  String command;
  if (Serial.available() > 0) {
    command = Serial.readString();
    Serial.flush();
    // convertCommand(command);
    readCommand(command);
  }
}

void readCommand(String instruction) {
  int curCount = 0, curId = 0;
  char curChar = instruction[0];
  while (curId < instruction.length()) {
    curChar = instruction[curId];
    while (curId + curCount < instruction.length() && instruction[curId + curCount] == curChar) 
      curCount++;

//    Serial.println(curCount);/
    
    switch (curChar) {
      case 'f': 
        forward(curCount); 
        // readSensors(true); 
        break;
      case 'b':
        backward(curCount); readSensors(true); break;
      case 'l':
        turnLeft(88 * curCount); readSensors(true); break;
      case 'r':
        turnRight(90 * curCount); readSensors(true); break;      
      case 's':
        readSensors(false); break;
      case 'a':
        adjust(); break;
      case 't': //test sensor
        readSingleSensor(L1, 101); break;  
    }
    
    curId += curCount;
    curCount = 0;
  }
  
}

void adjust() {  
  int maxIter = 0;
  while (maxIter < 10) {
    if (done) break;
    double left = readSingleSensor(L1, 99);
    double right = readSingleSensor(R2, 99);
  
    if (left < 0 || right < 0) return;
    
    double sensorDifference = left - right;
    //Serial.println(left);
    //Serial.println(right);
    //check dist between sensors
    int degreesToTurn = atan2(abs(sensorDifference), distBetweenSensors) / PI * 180;
    if (degreesToTurn > 25) continue;
    //Serial.println(degreesToTurn);
    //Serial.println(sensorDifference);
    if (abs(sensorDifference) < adjustEpsilon) {
      Serial.println("done");
      return;
    }
    if (sensorDifference > 0) {
      turnRight(degreesToTurn);
    }
    else {
      turnLeft(degreesToTurn);
    }  
    delay(10);
    maxIter++;
  }
}


double getOffset(int noGrid) {
  double offset;
  switch (noGrid) {
    case 1: offset = 0; break;
    default: offset = -0.3; break;
  }
  return offset;
}

void forward(int noGrid) {
//  Serial.println(noGrid);
  double offset = getOffset(noGrid);
  goDigitalDist(90, noGrid * 10 + offset, FORWARD, false);
  // goDigitalDiff(noGrid * 10 + offset, FORWARD);
}

void backward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(90, noGrid * 10 + offset, BACKWARD, false);
  //  goDigitalDiff(noGrid * 10 + offset, BACKWARD);
}

void turnLeft(double angle) {
  goDigitalDist(60, 2 * PI * robot_radius * angle/360, ROTATE_CCW, false);
}

void turnRight(double angle) {
  goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CW, false);

}

void doEncoderLeft() {
//  Serial.println(encoderLPos);
  if (encoderLPos < ticksDistance) {
    encoderLPos++;
  } else if (encoderLPos == ticksDistance && !(encoderLPos == 0 && ticksDistance == 0)) {
    brake();
  }
    
  currentPulseTime1 = micros() - prevPulseTime1;
  prevPulseTime1 = micros();
}

void doEncoderRight() {
  if (encoderRPos < ticksDistance) {
    encoderRPos++;
  } else if (encoderRPos == ticksDistance && !(encoderRPos == 0 && ticksDistance == 0)) {
    brake();
  }
  
  currentPulseTime2 = micros() - prevPulseTime2;
  prevPulseTime2 = micros();
}

//---------PID-----------//

double errorPrior = 0, integral = 0;
int leftPrevTicks = 0, rightPrevTicks = 0;
double kpl = 0.35, kil = 0.89, kdl = 0, kpr = 1.36, kir = 0, kdr = 0, kp = 1.6, ki = 0.3, kd = 0.6;
int leftRampupOffset, rightRampupOffset, rampUpDelay, leftBreak, rightBreak;


//---------------Testing----------------//


//ffff;p;k;k;k;k;k;k
//fffff:n:100:300

void convertCommand(String command) {
  String curString, split[15];
  int curIndex = 0, splitId = 0;
  while (curIndex < command.length()) {
    curString = "";
    int index = curIndex;
    while (index < command.length() && command[index] != ';') {
      curString += command[index];
      index++;
    }
    split[splitId] = curString;
    splitId++;
    curIndex = index + 1;
    curString = "";
  }

  if (split[1] == "p") {
    kpl = split[2].toFloat(); 
    kil = split[3].toFloat();
    kdl = split[4].toFloat();
    kpr = split[5].toFloat();
    kir = split[6].toFloat();
    kdr = split[7].toFloat();
    leftRampupOffset = split[8].toFloat();
    rightRampupOffset = split[9].toFloat();
    rampUpDelay = split[10].toFloat();
    leftBreak = split[11].toFloat();
    rightBreak = split[12].toFloat();
    readCommand(split[0]);
    
  } else if (split[1] == "n") {
    md.setSpeeds(split[2].toFloat(), split[3].toFloat());
  } else if (split[1] == "d") {
    kp = split[2].toFloat();
    ki = split[3].toFloat();
    kd = split[4].toFloat();
    leftRampupOffset = split[5].toFloat();
    rightRampupOffset = split[6].toFloat();
    rampUpDelay = split[7].toFloat();
    leftBreak = split[8].toFloat();
    rightBreak = split[9].toFloat();

    readCommand(split[0]);
  }
}


void goDigitalDist(double desired_rpm, float dist, int direction, bool sense) {  
  
  int LMag = 1, RMag = 1;

  if (direction == ROTATE_CW) {
    RMag = -1;
  } 
  else if (direction == ROTATE_CCW) {
    LMag = -1;
  } else if (direction == BACKWARD) {
    RMag = -1;
    LMag = -1;
  }

  ticksDistance = (int) (562.215 * dist / (2 * PI * wheel_radius));

//  Serial.println(ticksDistance);

  // Ramp up
   rampUp(LMag, RMag);
   
  // PID
//  double prevRpmL = 0, prevRpmR = 0;
//  while(1) {    
//    double leftRpm = getLeftRpm();
//    double rightRpm = getRightRpm();
//
//    Serial.print(leftRpm);
//    Serial.print(" pid ");
//    Serial.println(rightRpm);
//    
//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, leftRpm, kpl, kil, kdl);
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, rightRpm, kpr, kir, kdr);
//
//    double newLeftRpm = constrain(prevRpmL + leftDigitalPidOutput, 0, 140);
//    double newRightRpm = constrain(prevRpmR + rightDigitalPidOutput, 0, 140);
//
//    motorSetRpm(newLeftRpm, newRightRpm, LMag, RMag);
//
//    prevRpmL = newLeftRpm;
//    prevRpmR = newRightRpm;
//    delay(iteration_time * 1000);
//  }
}

double computeDigitalPid(double desired_value, double actual_value, double kp, double ki, double kd) {
  double error = desired_value - actual_value;
  integral += (error * iteration_time);
  double derivative = (error - errorPrior)/iteration_time;
  double output = kp * error + ki * integral + kd * derivative;
  errorPrior = error;
  return output;
}

void rampUp(int directionL, int directionR) {
  
  double currentLRampup = 0, currentRRampup = 0;

  double start_time = micros();
  
  while (currentLRampup < desired_rpm && currentRRampup < desired_rpm) { 
    double leftRpm = getLeftRpm();
    double rightRpm = getRightRpm();

    Serial.print(leftRpm);
    Serial.print(" rampup ");
    Serial.println(rightRpm);

    double diff_constraint = 2;
    double diff = leftRpm - rightRpm;
    if (abs(diff) <  diff_constraint) {
      currentLRampup += diff_constraint;
      currentRRampup += diff_constraint;
    }
    else if (diff > 0) currentRRampup += diff_constraint;
    else currentLRampup += diff_constraint;

    if (micros() < start_time + 40000) 
      currentRRampup = 0;
      
    motorSetRpm(currentLRampup, currentRRampup, directionL, directionR);
    delay(iteration_time * 1000);
  }
}

void motorSetRpm(float rpmL, float rpmR, int directionL, int directionR) {
  int speedL = rpmL != 0 ? ((float)rpmL + 4.1114) / 0.2705 : 0;
  int speedR = rpmR != 0 ? ((float)rpmR + 6.0446) / 0.2733 : 0;
  md.setSpeeds(speedL * directionL, speedR * directionR);
}

double getLeftRpm() {
  if (currentPulseTime1 == 0)
    return 0;
  return 60000 / (((currentPulseTime1) / 1000.0) * 562.215);
}

double getRightRpm() {
  if (currentPulseTime2 == 0)
    return 0;
  return 60000 / (((currentPulseTime2) / 1000.0) * 562.215);
}

void brake() {
  Serial.println("braking...");
  
  integral = 0;
  errorPrior = 0;
  encoderLPos = 0;
  encoderRPos = 0;
  ticksDistance = 0;

  md.setBrakes(400,400);
  md.setSpeeds(0,0);
}


//------------Sensor--------------//


void readSensors(bool returnGrid) {
  double l1 = readSingleSensor(L1, 17);
  double f3 = readSingleSensor(F3, 17);
  double r2 = readSingleSensor(R2, 17);
  double r1 = readSingleSensor(R1, 17);
  double f1 = readSingleSensor(F1, 17);
  double back = readSingleSensor(BACK, 17);
  String s;
  if (returnGrid) 
    s = stringifyGrid(l1) + ";" + stringifyGrid(back) + ";" + stringifyGrid(r2) + ";" + stringifyGrid(f3) + ";" + stringifyGrid(r1) + ";" + stringifyGrid(f1);
  else 
    s = stringify(l1) + ";" + stringify(back) + ";" + stringify(r2) + ";" + stringify(f3) + ";" + stringify(r1) + ";" + stringify(f1);
  Serial.println(s);
} 

String stringify(double value) {
  return String(value);
}

String stringifyGrid(double value) {
  if (value < 0) return "-1";
  if (value < 8.0) return "1";
  return String((int)(value/10 + 1.5));
}

double readSingleSensor(int sensorNumber, int readTimes) {
  // read the pin 7 times to get median value
  int sensorVal[readTimes];
  int sortedSensorVal[readTimes];
  for (int i = 0; i < readTimes; i++) {
    sensorVal[i] = analogRead(sensorNumber);   
    Serial.println(sensorVal[i]);      
    //delay(10);
  }

  int readValue = kthSmallest(sensorVal, 0 ,readTimes-1, readTimes/2);

  if (readValue <= 3) return -10;

  // deduce distance from value read from analog pin
  switch (sensorNumber) {
    case F1: 
      return 5452.08/(readValue - 3) - 1.246;
    case R1: 
      return 6534.06/(readValue - 3) - 3.83;
    case L1:
      Serial.println(6270.15/(readValue - 3) - 2.8977);
      return 6270.15/(readValue - 3) - 1.8977;
    case F3:
      return 14054.34/(readValue - 3) - 5.472;
    case BACK:
      return 6142.70/(readValue - 3) - 3.088;
    case R2:
      return 6725.72/(readValue - 3) - 4.077;
//      case R2:
//        return 6787 / (readValue - 3) - 4;
  }
}

int partition(int arr[], int l, int r)
{
    int x = arr[r], i = l;
    for (int j = l; j <= r - 1; j++) {
        if (arr[j] <= x) {
            swap(&arr[i], &arr[j]);
            i++;
        }
    }
    swap(&arr[i], &arr[r]);
    return i;
}

void swap(int* a, int* b) {
  int tmp = *a;
  *a = *b;
  *b = tmp;
}
 
// This function returns k'th smallest 
// element in arr[l..r] using QuickSort 
// based method.  ASSUMPTION: ALL ELEMENTS
// IN ARR[] ARE DISTINCT
int kthSmallest(int arr[], int l, int r, int k)
{
    // If k is smaller than number of 
    // elements in array
    if (k > 0 && k <= r - l + 1) {
 
        // Partition the array around last 
        // element and get position of pivot 
        // element in sorted array
        int index = partition(arr, l, r);
 
        // If position is same as k
        if (index - l == k - 1)
            return arr[index];
 
        // If position is more, recur 
        // for left subarray
        if (index - l > k - 1) 
            return kthSmallest(arr, l, index - 1, k);
 
        // Else recur for right subarray
        return kthSmallest(arr, index + 1, r, 
                            k - index + l - 1);
    }
 
    // If k is more than number of 
    // elements in array
    return 1000;
}

//void goDigitalDiff(double dist, int direction) {
//  int LMag = 1, RMag = 1;
//
//  if (direction == ROTATE_CW) {
//    RMag = -1;
//  } 
//  else if (direction == ROTATE_CCW) {
//    LMag = -1;
//  } else if (direction == BACKWARD) {
//    RMag = -1;
//    LMag = -1;
//  }
//
//  int start_ticks = encoderLPos;
//  int leftSpeed = 0, rightSpeed = 0;
//  
//  if (direction == FORWARD || direction == BACKWARD) {
//    for (int i = 20; i < 300; i += 10) {
//      leftSpeed = LMag * (i + leftRampupOffset);
//      rightSpeed = RMag * (i + rightRampupOffset);
//      md.setSpeeds(leftSpeed, rightSpeed);
//      delay(rampUpDelay);
//    }
//  } 
////  Serial.println(encoderLPos);
//  
//  leftPrevTicks = encoderLPos;
//  rightPrevTicks = encoderRPos;
//
//  delay(iteration_time * 1000);
//
//  int totalTicks = (int) (562.25 * dist / (2 * PI * wheel_radius));
//
//  while(1) {
//    // Serial.println(encoderLPos);
//    if (encoderLPos >= start_ticks + totalTicks) 
//      break;
//      
////    double pidOutput = abs(computeDigitalPid(0, /ticksToRpm((encoderLPos - leftPrevTicks) - (encoderRPos - rightPrevTicks), iteration_time), kp, ki, kd));
//    double pidOutput = computeDigitalPid(0, -encoderLPos + encoderRPos, kp, ki, kd);
//
//    leftSpeed = constrain(leftSpeed - pidOutput/50, 0, 255) / 255 * 400;
//    rightSpeed = constrain(rightSpeed + pidOutput/50, 0, 255) / 255 * 400;
//    
////    if (leftSpeed > rightSpeed) {
////      leftSpeed = constrain(abs(leftSpeed) - pidOutput, 0 , 400) * LMag;
////      rightSpeed = constrain(abs(rightSpeed) + pidOutput, 0 , 400) * RMag;
////    } else {
////      leftSpeed = constrain(abs(leftSpeed) + pidOutput, 0 , 400) * LMag;
////      rightSpeed = constrain(abs(rightSpeed) - pidOutput, 0 , 400) * RMag;
////    }
//    Serial.print(pidOutput);
//    Serial.print(" ");
//    Serial.print(leftSpeed);
//    Serial.print(" ");
//    Serial.println(rightSpeed);
//    md.setSpeeds(leftSpeed, rightSpeed);
//    leftPrevTicks = encoderLPos;
//    rightPrevTicks = encoderRPos;
//    delay(iteration_time * 1000); // iteration_time in seconds
//  }
//  integral = 0;
//  errorPrior = 0;
//  //100, 300 for lab2 lounge 100,250
//  if (direction == FORWARD || direction == BACKWARD)
//    md.setBrakes(leftBreak, rightBreak);
//  else 
//    md.setBrakes(200, 300);  
//}


    //1 grid
//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 7.5, 6.0, 0);
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.5, 0);
    


    //double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 5.0, 5.75, 0);
    //double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.25, 0);
    
    //7.75
//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 5.225, 4.75, 0);  
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.75, 4.5, 0);

//    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), 4.875, 5.75, 0.08);
//    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), 7.25, 4.5, 0);


