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
const double leftTurnAngle = 87;
const double rightTurnAngle = 82;

volatile int encoderRPos = 0;
int encoderRPinALast = LOW;

volatile int encoderLPos = 0;
int encoderLPinALast = LOW;

double prevError = 0;

DualVNH5019MotorShield md;

int ticksDistance = 0;
double currentPulseTime1 = 0, prevPulseTime1 = 0;
double currentPulseTime2 = 0, prevPulseTime2 = 0;
double desired_rpm = 80;
bool running;

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
    if (maxIter == 5) break;
    double distBetweenSensors;

    if (isFront) {
      left = readSingleSensor(L1, 51) - 5.4;
      right = readSingleSensor(R2, 51) - 5.3;
      distBetweenSensors = 18;
    }
    else {
      left = readSingleSensor(R1, 51) + 0.3;
      right = readSingleSensor(F1, 51);
      distBetweenSensors = 18.0;
    }
    if (left < 0 || right < 0) return;

    double sensorDifference = left - right;
    if (abs(sensorDifference) > 10) return;
    //    Serial.println(left);
    //    Serial.println(right);
    //check dist between sensors
    double degreesToTurn = atan2(abs(sensorDifference), distBetweenSensors) / PI * 180;
//    Serial.println(degreesToTurn);
    if (degreesToTurn > 25) break;
    degreesToTurn = constrain(degreesToTurn, 0, 5);
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
  //  else {
  //    if (abs(meanSensorReading - 11.8) > 3) {
  //      turnLeft(leftTurnAngle);
  //      delay(1000);
  //      adjust(true);
  //      delay(1000);
  //      turnRight(rightTurnAngle);
  //    }
  //  }
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
  double offset;
  switch (noGrid) {
    case 1: offset = 0; break;
    case 2: offset = -1.75; break;
  }
  return offset;
}

void forward(int noGrid) {
  //  Serial.println(noGrid);
  double offset = getOffset(noGrid);
  goDigitalDist(noGrid * 10 + offset, FORWARD, false);
  // goDigitalDiff(noGrid * 10 + offset, FORWARD);
}

void backward(int noGrid) {
  double offset = getOffset(noGrid);
  goDigitalDist(noGrid * 10 + offset, BACKWARD, false);
  //  goDigitalDiff(noGrid * 10 + offset, BACKWARD);
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

//---------PID-----------//

double errorPriorL = 0, integralL = 0, errorPriorR = 0, integralR = 0;
int leftPrevTicks = 0, rightPrevTicks = 0;
//double kpl = 0.35, kil = 0.89, kdl = 0, kpr = 1.36, kir = 0, kdr = 0, kp = 1.6, ki = 0.3, kd = 0.6;
//double kpl = 0.35, kil = 0.20, kdl = 0.01, kpr = 0.36, kir = 0.20, kdr = 0.008, kp = 1.6, ki = 0.3, kd = 0.6; // for short distance
//0.3;0.29;0.02;0.75;0.3;0.015;60
double kpl = 0.3, kil = 0.29, kdl = 0.02, kpr = 0.75, kir = 0.3, kdr = 0.01, kp = 1.6, ki = 0.3, kd = 0.6; // for short distance

int leftRampupOffset, rightRampupOffset, rampUpDelay, leftBreak, rightBreak, offset = 0;


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
    offset = split[9].toFloat();
//    leftRampupOffset = split[8].toFloat();
//    rightRampupOffset = split[9].toFloat();
//    rampUpDelay = split[10].toFloat();
//    leftBreak = split[11].toFloat();
//    rightBreak = split[12].toFloat();
    desired_rpm = split[8].toFloat();
    readCommand(split[0]);

  } else if (split[1] == "n") {
    md.setSpeeds(split[2].toFloat(), split[3].toFloat());
    delay(2000);
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
  }
}


void goDigitalDist(float dist, int direction, bool sense) {
  // Serial.println("start going...");
  // Serial.println(dist);
//  Serial.println(kpl);
//  Serial.println(kil);
//  Serial.println(kdl);

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

  double diff_constraint = 5;
//  double desired_rpm = 60;
  if (dist < 6) {
    desired_rpm = 60;
    diff_constraint = 5;
  }

  if (RMag * LMag == -1) {
    diff_constraint = 5;
  }

  ticksDistance = (int) (562.215 * dist / (2 * PI * wheel_radius));

  // Ramp up
//    rampUp(desired_rpm, diff_constraint, LMag, RMag);
//    brake();
//    delay(500);
  // goConstSpeed(desired_rpm, desired_rpm, LMag, RMag);  
//     motorSetRpm(desired_rpm, desired_rpm + offset, LMag, RMag);
//  motorSetRpm(desired_rpm - 40, desired_rpm, LMag, RMag);
//  delay(offset);
  md.setSpeeds(347 * LMag, 372 * RMag);
//    motorSetRpm(desired_rpm, desired_rpm, LMag, RMag);
       if (dist < 15) delay(400);
       else delay(550);

//  while (running) {
//    if (abs(desired_rpm - getLeftRpm()) < 1) 
//      break;
//    Serial.print("");
//    Serial.print(getLeftRpm());
//    Serial.print(" ramp ");
//    Serial.println(getRightRpm());
//  }

//f;p;0.35;0.2;0.02;0.4;0.2.0;0.01;60
  // PID
//  double prevRpmL = 0, prevRpmR = 0;
//  double startTime = millis();
    double prevRpmL = getLeftRpm(), prevRpmR = getRightRpm();
//  Serial.println(integralL + integralR);
//  Serial.println(errorPriorL + errorPriorR);
  while(running) {
    double leftRpm = getLeftRpm();
    double rightRpm = getRightRpm();
//
    Serial.print("");
//    double start_time = millis();
//    Serial.print(leftRpm);
//    Serial.print(" pid ");
//    Serial.println(rightRpm);
//    Serial.println(millis() - start_time);

    double leftDigitalPidOutput = computeDigitalPid(desired_rpm, leftRpm, kpl, kil, kdl, true);
    double rightDigitalPidOutput = computeDigitalPid(desired_rpm, rightRpm, kpr, kir, kdr, false);
    
//    Serial.println(leftDigitalPidOutput);
//    Serial.print(" pid ");
//    Serial.println(rightDigitalPidOutput);
    
    double newLeftRpm = constrain(prevRpmL + leftDigitalPidOutput, 0, 140);
    double newRightRpm = constrain(prevRpmR + rightDigitalPidOutput, 0, 140);

//    Serial.println(newLeftRpm);
//    Serial.print(" pid ");
//    Serial.println(newRightRpm);

    if (running) motorSetRpm(newLeftRpm, newRightRpm, LMag, RMag);

    prevRpmL = newLeftRpm;
    prevRpmR = newRightRpm;
    delay(iteration_time);
  }
  brake();
//  Serial.print(millis() - startTime);
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

void rampUp(double desired_rpm, double diff_constraint, int directionL, int directionR) {

  double currentLRampup = 0, currentRRampup = 0;

  //  double start_time = micros();

  //  Serial.println(desired_rpm);
  double leftRpm = 0;
  double rightRpm = 0;
  while (running && leftRpm < desired_rpm && rightRpm < desired_rpm) {

    // Serial.println("inside ramp up...");

    leftRpm = getLeftRpm();
    rightRpm = getRightRpm();

    //    Serial.print(leftRpm);
    //    Serial.print(" rampup ");
    //    Serial.println(rightRpm);

    double diff = leftRpm - rightRpm;

    if (abs(diff) <  diff_constraint) {
      currentLRampup += diff_constraint;
      currentRRampup += diff_constraint;
    }
    else if (diff > 0) {
      currentRRampup += diff / 2;
      currentLRampup -= diff / 2;
    }
    else {
      currentLRampup -= diff / 2;
      currentRRampup += diff / 2;
    }

    //    if (micros() < start_time + 40000)
    //      currentRRampup = 0;

    if (running)
      motorSetRpm(currentLRampup, currentRRampup, directionL, directionR);
    delay(iteration_time);
  }
  // Serial.println("outside ramp up...");

  //  Seria/l.print(leftRpm);
  //  Serial.p/rint(" after ");
  //  Serial.prin/tln(rightRpm);

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
}

double getRightRpm() {
  if (currentPulseTime2 == 0)
    return 0;
  return 60000 / (((currentPulseTime2) / 1000.0) * 562.215);
}

void brake() {
  // Serial.println("braking...");


  md.setBrakes(400, 380);
  // md.setSpeeds(0,0);

  //  readSensors(true);

  resetMove();  
}

void resetMove() {
  integralL = 0;
  errorPriorL = 0;
  integralR = 0;
  errorPriorR = 0;
  encoderLPos = 0;
  encoderRPos = 0;
  ticksDistance = 0;
  running = false;
}


//------------Sensor--------------//


void readSensors(bool returnGrid) {
  double l1 = readSingleSensor(L1, 21) - 5.1;
  double f3 = readSingleSensor(F3, 21) - 14.5;
  double r2 = readSingleSensor(R2, 21) - 5.3;
  double r1 = readSingleSensor(R1, 21) - 7.7;
  double f1 = readSingleSensor(F1, 21) - 7.3;
  double back = readSingleSensor(BACK, 21) - 2.1;
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
    //    Serial.println(sensorVal[i]);
    //delay(10);
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
      //      Serial.println(6270.15/(readValue - 3) - 2.8977);
      // return (1 / (0.0002391473 * readValue - 0.0100251467));
      return 6270.15 / (readValue - 3) - 2.8977;
    case F3:
      return 14054.34 / (readValue - 3) - 5.472;
    case BACK:
      return 6142.70 / (readValue - 3) - 3.088;
    // return (1 / (0.0002391473 * readValue - 0.0100251467));
    case R2:
      return 6725.72 / (readValue - 3) - 4.077;
      // return (1 / (0.0002391473 * readValue - 0.0100251467));
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


