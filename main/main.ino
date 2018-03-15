#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <string.h>

//with small battery, kp left = 13, kp right = 20
//with big battery, 17;1.5;0;20;2;0;26;26;10;100;250 can go 10 squares
#define L1 A0
#define F3 A1
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

int n = LOW;
DualVNH5019MotorShield md;
long long Time;

int leftTickPrevError = 0, leftTickPrevPrevError = 0, rightTickPrevError = 0, rightTickPrevPrevError = 0;
int leftPrevOutput = 0, rightPrevOutput = 0;

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

int counter = 0;
int ticks[111];
void loop() {
//  if (!done) {
//    String command = "fffff;d;0.0005;0;0;0;0;0;100;100";
//    convertCommand(command);
//    done = true;
//  } else {
//    md.setSpeeds(0,0);
//  }
  
  String command;
  if (Serial.available() > 0) {
    command = Serial.readString();
    Serial.flush();
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

    switch (curChar) {
      case 'f': 
        forward(curCount); break;
      case 'b':
        backward(curCount); break;
      case 'l':
        turnLeft(90 * curCount); break;
      case 'r':
        turnRight(90 * curCount); break;      
      case 's':
        readSensors(true); break;
      case 'a':
        Serial.println(curCount);
        for (int i = 0; i < curCount; i++) {
          adjust(); 
        }
        break;
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
  double offset = getOffset(noGrid);
  //goDigitalDist(70, noGrid * 10 + offset, FORWARD, false);
  goDigitalDiff(noGrid * 10 + offset, FORWARD, false);
  readSensors(true);
}

void backward(int noGrid) {
  double offset = getOffset(noGrid);
  //goDigitalDist(70, noGrid * 10 + offset, BACKWARD, false);
    goDigitalDiff(noGrid * 10 + offset, BACKWARD, false);
  readSensors(true);
}

void turnLeft(double angle) {
//  goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CCW, false);
  goDigitalDiff(2*PI*robot_radius * angle/360, ROTATE_CCW, false);
  readSensors(true);
}

void turnRight(double angle) {
  //goDigitalDist(60, 2*PI*robot_radius * angle/360, ROTATE_CW, false);
  goDigitalDiff(2*PI*robot_radius * angle/360, ROTATE_CW, false);
  readSensors(true);
}

void doEncoderLeft() {
  encoderLPos++;
}

void doEncoderRight() {
  encoderRPos++;
}

//---------PID-----------//

double errorPrior = 0, integral = 0;
int leftPrevTicks = 0, rightPrevTicks = 0;
double kpl = 7.0, kil = 0, kdl = 0, kpr = 6.44, kir = 0, kdr = 0, kp = 1.6, ki = 0.3, kd = 0.6;
int leftRampupOffset, rightRampupOffset, rampUpDelay, leftBreak, rightBreak, iniLeftSpeed = 271, iniRightSpeed = 300;


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
    delay(3000);
    md.setBrakes(split[4].toFloat(), split[5].toFloat());
    Serial.println(encoderLPos);
    Serial.println(encoderRPos);
    delay(100);
    md.setSpeeds(-split[2].toFloat(), -split[3].toFloat());
    delay(3000);
    md.setBrakes(split[4].toFloat(), split[5].toFloat());
    Serial.println(encoderLPos);
    Serial.println(encoderRPos);
    encoderLPos = 0; 
    encoderRPos = 0;
  } else if (split[1] == "d") {
    if (split[2] == "no") {
      iniLeftSpeed = split[3].toFloat();
      iniRightSpeed = split[4].toFloat();      
    } else {
      kp = split[2].toFloat();
      ki = split[3].toFloat();
      kd = split[4].toFloat();
      leftRampupOffset = split[5].toFloat();
      rightRampupOffset = split[6].toFloat();
      rampUpDelay = split[7].toFloat();
      leftBreak = split[8].toFloat();
      rightBreak = split[9].toFloat();
  
    }
    
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

  int start_ticks = encoderLPos;

  if (direction == FORWARD || direction == BACKWARD) {
    leftPrevTicks = encoderLPos;
    int prev = encoderLPos;
    for (int i = 20; i < 300; i += 10) {
      md.setSpeeds(LMag * (i - leftRampupOffset), RMag * (i + rightRampupOffset));
      delay(rampUpDelay);
    }
  } 
  int leftSpeed = 0, rightSpeed = 0;
  
  leftPrevTicks = encoderLPos;
  rightPrevTicks = encoderRPos;

  delay(iteration_time * 1000);

  int totalTicks = (int) (562.25 * dist / (2 * PI * wheel_radius));
 
  while(1) {
    if (encoderLPos >= start_ticks + totalTicks) break;

    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderLPos - leftPrevTicks, iteration_time), kpl, kil, kdl);
    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, ticksToRpm(encoderRPos - rightPrevTicks, iteration_time), kpr, kir, kdr);

    double newLeftSpeed = constrain(abs(leftSpeed) + leftDigitalPidOutput, 0 , 400) * LMag;
    double newRightSpeed = constrain(abs(rightSpeed) + rightDigitalPidOutput, 0 , 400) * RMag;
    md.setSpeeds(newLeftSpeed, newRightSpeed);
//    md.setSpeeds(newLeftSpeed, 0);
//    md.setSpeeds(0, newRightSpeed);
    
    leftPrevTicks = encoderLPos;
    rightPrevTicks = encoderRPos;

    if (sense) {  
      double d1 = readSingleSensor(R2, 17);
      double d2 = readSingleSensor(L1, 17);
      
      if ((d1 <= 13.5 && d1 >= 10) || (d2 <= 13.5 && d2 >= 10)) break;
    }
    
    delay(iteration_time * 1000); // iteration_time in seconds
  }
  integral = 0;
  errorPrior = 0;
  //100, 300 for lab2 lounge 100,250
  if (direction == FORWARD || direction == BACKWARD)
    md.setBrakes(leftBreak, rightBreak);
  else 
    md.setBrakes(200, 300);  
  //rampDown();
}

// ffff;d;0.5;0;0;0;0;0;100;100
// ffff;n;100;100;100;100
void goDigitalDiff(double dist, int direction, bool usePid) {
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

  int start_ticks = encoderLPos;
  int leftSpeed = iniLeftSpeed, rightSpeed = iniRightSpeed;

  leftPrevTicks = encoderLPos;
  rightPrevTicks = encoderRPos;
  
  delay(iteration_time * 1000);

  int totalTicks = (int) (562.25 * dist / (2 * PI * wheel_radius));

  if (usePid) {
    while(1) {
      // Serial.println(encoderLPos);
      if (encoderLPos >= start_ticks + totalTicks || encoderRPos >= start_ticks + totalTicks) 
        break;
        
      double pidOutput = computeDigitalPid(0, encoderLPos - encoderRPos, kp, ki, kd);
  
  //    leftSpeed = constrain(leftSpeed + pidOutput, 0, 400);
  //    rightSpeed = constrain(rightSpeed - pidOutput, 0, 400);
  
      if (encoderLPos >= encoderRPos) {
        leftSpeed = constrain(leftSpeed - abs(pidOutput), 0, 400);
      } else {
        leftSpeed = constrain(leftSpeed + abs(pidOutput), 0, 400);
      }
      
      //Serial.println(pidOutput);
      //Serial.print(encoderLPos);
      //Serial.print(" ");
      //Serial.println(encoderRPos);
      //Serial.print(leftSpeed);
      //Serial.print(" ");
      //Serial.println(rightSpeed);
      md.setSpeeds(leftSpeed, rightSpeed);
      leftPrevTicks = encoderLPos;
      rightPrevTicks = encoderRPos;
      delay(iteration_time * 1000); // iteration_time in seconds
    }
  
    if (direction == FORWARD || direction == BACKWARD)
      md.setBrakes(leftBreak, rightBreak);
    else 
      md.setBrakes(200, 300);  
  
    integral = 0;
    errorPrior = 0;
    encoderLPos = 0;
    encoderRPos = 0;
  }
  else {
    md.setSpeeds(leftSpeed * LMag, rightSpeed * RMag);
    while (1) {
      if (encoderLPos >= start_ticks + totalTicks || encoderRPos >= start_ticks + totalTicks) 
        break;
    }
    md.setSpeeds(0,0);
    encoderLPos = 0;
    encoderRPos = 0;
  }
}

int id = 1;
double computeDigitalPid(double desired_value, double actual_value, double kp, double ki, double kd) {
  //Serial.print(millis());
  //Serial.print(" ");
  //Serial.print("actual ");
  //Serial.println(actual_value);
  double error = desired_value - actual_value;
  integral += (error * iteration_time);
  double derivative = (error - errorPrior)/iteration_time;
  double output = kp * error + ki * integral + kd * derivative;
  errorPrior = error;
  //Serial.println(output);
  return output;
}

double ticksToRpm(int tickCount, double period) { // period in seconds
  return 60 / (562.25 / ((tickCount) / period));
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
    //Serial.println(sensorVal[i]);      
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
      return 6270.15/(readValue - 3) - 2.8977;
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



