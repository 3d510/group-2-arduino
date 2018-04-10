#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include <string.h>
//0.4;0.8;0.02775;0.5;0.75;0.02 decent bluetack battery, not fully charged
//0.3;0.8;0.02775;0.5;0.75;0.02 for no bluetack battery, not fully charged
// ;p;0.35;0.2;0.01;0.4;0.2.0;0.008;60 -- no bluetack battery
//with small battery, kp left = 13, kp right = 20
//with big battery, 17;1.5;0;20;2;0;26;26;10;100;250 can go 10 squares`



// newest: long distance: fffff;p;0.33;0.15;0.01;0.52;0.24;0.01;60
// newest newest: long dist: f;p;0.75;0.25;0.02;0.6;0.24;0.01;60
// f;p;0.43;0.15;0.02;0.52;0.38;0.02;60
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
const double iteration_time = 10;
const double wheel_radius = 3; // cm
const double robot_radius = 9.5; // cm
const double adjustEpsilon = 0.15;
//blutack
//const double leftTurnAngle = 85;
//const double rightTurnAngle = 88;

//////no blutack
// double leftTurnAngle = 87;
// double rightTurnAngle = 89;

double leftTurnAngle = 79;
double rightTurnAngle = 78;
double oneGridOffset = 0;

//const double leftTurnAngle = 85;
//const double rightTurnAngle = 89;

volatile int encoderRPos = 0;
int encoderRPinALast = LOW;

volatile int encoderLPos = 0;
int encoderLPinALast = LOW;

double prevError = 0;
int leftForward = 345;
int rightForward = 372;

DualVNH5019MotorShield md;

int ticksDistance = 0;
double currentPulseTime1 = 0, prevPulseTime1 = 0;
double currentPulseTime2 = 0, prevPulseTime2 = 0;
double desired_rpm = 80;
bool running;

//---------PID-----------//

double errorPriorL = 0, integralL = 0, errorPriorR = 0, integralR = 0;
int leftPrevTicks = 0, rightPrevTicks = 0;
//double kpl = 0.16, kil = 0.014, kdl = 0.032, kpr = 0.97, kir = 0.87, kdr = 0.01, kp = 1.6, ki = 0.3, kd = 0.6; // for short distance blutack
//double kpl = 0.08, kil = 0.014, kdl = 0.032, kpr = 0.97, kir = 0.87, kdr = 0.01, kp = 1.6, ki = 0.3, kd = 0.6; // for short distance no blutack
//double kpl = 0.0325, kil = 0.014, kdl = 0.032, kpr = 0.97, kir = 0.87, kdr = 0.01, kp = 1.6, ki = 0.3, kd = 0.6; // for short distance blutack
//0.08;0.014;0.032;0.5;0.13;0.01;80;5
//double kpl = 0.08, kil = 0.014, kdl = 0.032, kpr = 0.5, kir = 0.13, kdr = 0.01, kp = 1.6, ki = 0.3, kd = 0.6;
//2.52;10;0.1;10;10;0.1;70;200
double kpl = 2.52, kil = 10, kdl = 0.1, kpr = 10, kir = 10, kdr = 0.1, kp = 1.6, ki = 0.3, kd = 0.6;
int leftRampupOffset, rightRampupOffset, rampUpDelay, leftBreak, rightBreak, delay_offset = 0;

//--------Main---------//

void setup() {
  pinMode (encoderLPinA, INPUT);
  pinMode (encoderLPinB, INPUT);
  pinMode (encoderRPinA, INPUT);
  pinMode (encoderRPinB, INPUT);
  running = false;
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
   convertCommand(command);
//    readCommand(command);
  }
}

void readCommand(String instruction) {
  //  Serial.println(instruction);

  int curCount = 0, curId = 0;
  char curChar = instruction[0];
  while (curId < instruction.length()) {
    curChar = instruction[curId];
    while (curId + curCount < instruction.length() && instruction[curId + curCount] == curChar)
      curCount++;

    switch (curChar) {
      case 'f':
        forward(curCount);
        readSensors(true);
        break;
      case 'b':
        backward(curCount); readSensors(true);
        break;
      case 'l':
        turnLeft(leftTurnAngle * curCount);
        readSensors(true);
        break;
      case 'r':
        turnRight(rightTurnAngle * curCount);
        readSensors(true);
        break;
      case 's':
        readSensors(true); break;
      case 'x': //front
        adjust(true);
        readSensors(true);
        break;
      case 'y': //left
        adjust(false);
        readSensors(true);
        break;
      case 'z':
        readSensors(false); break;          
    }

    curId += curCount;
    curCount = 0;
  }

}

void adjust(bool isFront) {
  // Serial.println("start calibrating..");
  int maxIter = 0;
  double left;
  double right;
  while (1) {
    if (maxIter >= 8) break;
    double distBetweenSensors;

    if (isFront) {
      left = readSingleSensor(L1, 51) - 5.4;
      right = readSingleSensor(R2, 51) - 5.3;
      distBetweenSensors = 18;
    }
    else {
      left = readSingleSensor(R1, 51) - 7.3;
      right = readSingleSensor(F1, 51) - 7.7;
      distBetweenSensors = 18.0;
    }
    if (left < 0 || right < 0) return;

    double sensorDifference = left - right;
    if (abs(sensorDifference) > 10) return;
    //    Serial.println(left);
    //    Serial.println(right);
    //check dist between sensors
    double degreesToTurn = atan2(abs(sensorDifference), distBetweenSensors) / PI * 180;
//    if (degreesToTurn > 25) break;
    degreesToTurn = constrain(degreesToTurn, 0, 2);
    // Serial.println(sensorDifference);
    if (abs(sensorDifference) < adjustEpsilon) {
      break;
    }
    // Serial.println(degreesToTurn);
    if (sensorDifference > 0) {
      turnRight(degreesToTurn);
    }
    else {
      turnLeft(degreesToTurn);
    }
    maxIter++;
  }

  //      Serial.println("done");

  double meanSensorReading = (left + right) / 2;
  if (isFront) {
    adjustDist(meanSensorReading);
  }
  else if (abs(meanSensorReading) >= 7 && abs(meanSensorReading) <= 12) {
    turnLeft(leftTurnAngle);
    delay(200);
    adjust(true);
    delay(200);
    turnRight(rightTurnAngle);
  }
  integralL = 0;
  integralR = 0;
}

void adjustDist(double distToWallFront) {
  //  Serial.print("---");
  //  Serial.println(distToWallFront);

  if (distToWallFront > 13) return;

  double distDiff = distToWallFront - 5.1;
  int dir;
  if (distDiff > 0)
    dir = FORWARD;
  else
    dir = BACKWARD;

  if (abs(distDiff) <= adjustEpsilon) return;
  goDigitalDist(abs(distDiff), dir, false);

}

double getOffset(int noGrid) {
  double offset = 0;
  switch (noGrid) {
    case 1: offset = oneGridOffset; break;
    case 2: offset = 0; break;
  }
  return offset;
}

void forward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(noGrid * 10 + offset, FORWARD, false);
}

void backward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(noGrid * 10 + offset, BACKWARD, false);
}

void turnLeft(double angle) {
  goDigitalDist(2 * PI * robot_radius * angle / 360, ROTATE_CCW, false);
}

void turnRight(double angle) {
  goDigitalDist(2 * PI * robot_radius * angle / 360, ROTATE_CW, false);
}

void doEncoderLeft() {
  if (encoderLPos <= ticksDistance && !(encoderLPos == 0 && ticksDistance == 0)) {
    encoderLPos++;
  } else if (encoderLPos > ticksDistance) {
    brake();
  }
  currentPulseTime1 = micros() - prevPulseTime1;
  prevPulseTime1 = micros();
}

void doEncoderRight() {
  if (encoderRPos <= ticksDistance && !(encoderRPos == 0 && ticksDistance == 0)) {
    encoderRPos++;
  } else if (encoderRPos > ticksDistance) {
    brake();
  }

  currentPulseTime2 = micros() - prevPulseTime2;
  prevPulseTime2 = micros();
}


//---------------Testing----------------//

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

  if (splitId == 1) {
    readCommand(split[0]);
    return;
  }

  if (split[1] == "p") {
    kpl = split[2].toFloat();
    kil = split[3].toFloat();
    kdl = split[4].toFloat();
    kpr = split[5].toFloat();
    kir = split[6].toFloat();
    kdr = split[7].toFloat();
    delay_offset = split[9].toFloat();
    desired_rpm = split[8].toFloat();
    readCommand(split[0]);

  } else if (split[1] == "n") {
    md.setSpeeds(split[2].toFloat(), split[3].toFloat());
    delay(4000);
    brake();
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
  } else if (split[1] == "m") {
    leftTurnAngle = split[2].toFloat();
    rightTurnAngle = split[3].toFloat();
    oneGridOffset = split[4].toFloat();
    leftForward = split[5].toFloat();
    rightForward = split[6].toFloat();
  }
}


void goDigitalDist(float dist, int direction, bool sense) {

  resetMove();
  running = true;

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


//  double diff_constraint = 3;
  if (dist < 6) {
    desired_rpm = 60;
  }
  if (dist < 15 && direction == FORWARD) {
    delay_offset = 1000;
  }
  else delay_offset = 200;

  ticksDistance = (int) (562.215 * dist / (2 * PI * wheel_radius));

  md.setSpeeds(LMag * leftForward, RMag * rightForward);
  delay(delay_offset);
//f;p;0.35;0.2;0.02;0.4;0.2.0;0.01;60
  // PID
  double prevRpmL = getLeftRpm(), prevRpmR = getRightRpm();
//  double current_desired = desired_rpm; 
//  int counter = 0;
  while(running) {
    double leftRpm = getLeftRpm();
    double rightRpm = getRightRpm();

    Serial.print("");
//    if (counter++ % 3 == 0 && current_desired < desired_rpm) current_desired += 15;

    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, leftRpm, kpl, kil, kdl, true);
    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, rightRpm, kpr, kir, kdr, false);
    
    double newLeftRpm = constrain(prevRpmL + leftDigitalPidOutput, 0, 140);
    double newRightRpm = constrain(prevRpmR + rightDigitalPidOutput, 0, 140);

    if (running) motorSetRpm(newLeftRpm, newRightRpm, LMag, RMag);

    prevRpmL = newLeftRpm;
    prevRpmR = newRightRpm;
    delay(iteration_time);
  }
  brake();
//  delay(30);
//  delay(2000);
}

double computeDigitalPid(double desired_value, double actual_value, double Kp, double Ki, double Kd, bool isLeft) {
  double error = desired_value - actual_value;
  if (isLeft)
    integralL += (error * iteration_time / 1000.0);
  else
    integralR += (error * iteration_time / 1000.0);  
  double derivative = (error - (isLeft ? errorPriorL : errorPriorR)) / (iteration_time / 1000.0);
  double output = Kp * error + Ki * (isLeft ? integralL : integralR) + Kd * derivative;

  if (isLeft)
    errorPriorL = error;
  else 
    errorPriorR = error;
  return output;
}

void goConstSpeed(double rpm1, double rpm2, int directionL, int directionR) {
  while (running) {
    Serial.print("");
    motorSetRpm(rpm1, rpm2, directionL, directionR);
    delay(iteration_time);
  }
  Serial.println("outside");
}

void motorSetRpm(float rpmL, float rpmR, int directionL, int directionR) {
//  int speedL = rpmL != 0 ? ((float)rpmL + 4.1114) / 0.2705 : 0;
//  int speedR = rpmR != 0 ? ((float)rpmR + 6.0446) / 0.2733 : 0;
  int speedL = rpmL != 0 ? (float)rpmL * 4.336 + 35.517 : 0;
  int speedR = rpmR != 0 ? (float)rpmR * 4.546 + 34.064 : 0;
  md.setSpeeds(speedL * directionL, speedR * directionR);
}

double getLeftRpm() {
  if (currentPulseTime1 == 0)
    return 0;
  return 60000 / (((currentPulseTime1) / 1000.0) * 562.215);
//  double p = pulseIn(encoderLPinA, HIGH) * 2;
//  if (p == 0) return 0;
////  Serial.println(60000 / ((p / 1000.0) * 562.215));
//  return 60000 / ((p / 1000.0) * 562.215);
}

double getRightRpm() {
  if (currentPulseTime2 == 0)
    return 0;
  return 60000 / (((currentPulseTime2) / 1000.0) * 562.215);
//  double p = pulseIn(encoderRPinA, HIGH) * 2;
//  if (p == 0) return 0;
//  return 60000 / ((p / 1000.0) * 562.215);
}

void brake() {

  md.setBrakes(400, 400);
  delay(5);
  resetMove();
}

void resetMove() {
//  integralL = 0;
  errorPriorL = 0;
//  integralR = 0;
  errorPriorR = 0;
  encoderLPos = 0;
  encoderRPos = 0;
  ticksDistance = 0;
  running = false;
}


//------------Sensor--------------//


void readSensors(bool returnGrid) {
  double l1 = readSingleSensor(L1, 17) - 5.1;
  double f3 = readSingleSensor(F3, 17) - 14.5;
  double r2 = readSingleSensor(R2, 17) - 5.3;
  double r1 = readSingleSensor(R1, 17) - 7.7;
  double f1 = readSingleSensor(F1, 17) - 7.3;
  double back = readSingleSensor(BACK, 17) - 2.1;
  String s;
  if (returnGrid)
    s = stringifyGrid(l1, false) + ";" + stringifyGrid(back, false) + ";" + stringifyGrid(r2, false) + ";" + stringifyGrid(f3, true) + ";" + stringifyGrid(r1, false) + ";" + stringifyGrid(f1, false);
  else
    s = stringify(l1) + ";" + stringify(back) + ";" + stringify(r2) + ";" + stringify(f3) + ";" + stringify(r1) + ";" + stringify(f1);
  Serial.println(s);
}

String stringify(double value) {
  return String(value);
}

String stringifyGrid(double value, bool isLong) {
  if (isLong && value > 0 && value <= 60) {
    return String(int(value / 10.0) + 1);
  }
  if (value < 0 || value > 23)
    return "-1";
  if (value <= 13)
    return "1";
  else if (value <= 23)
    return "2";
}

double readSingleSensor(int sensorNumber, int readTimes) {
  int temp_value[7];
  int sensorVal[readTimes];
  int sortedSensorVal[readTimes];
  for (int i = 0; i < readTimes; i++) {
    for (int j = 0; j < 7; j++) 
      temp_value[j] = analogRead(sensorNumber);
    
    sensorVal[i] = kthSmallest(temp_value, 0, 6, 3);
  }

  int readValue = kthSmallest(sensorVal, 0 , readTimes - 1, readTimes / 2);

  if (readValue <= 3) return -10;

  return convertAdcToCent(sensorNumber, readValue);

}

double convertAdcToCent(int sensorNumber, double readValue) {
  // deduce distance from value read from analog pin
  switch (sensorNumber) {
    case F1:
      return 5452.08 / (readValue - 3) - 1.246;
    case R1:
      return 6534.06 / (readValue - 3) - 3.83;
    case L1:
      return 6270.15 / (readValue - 3) - 2.8977;
    case F3:
      return 14054.34 / (readValue - 3) - 5.472;
    case BACK:
      return 6142.70 / (readValue - 3) - 3.088;
    case R2:
      return 6725.72 / (readValue - 3) - 4.077;
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
