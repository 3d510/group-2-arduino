#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"

int val;
int encoder0PinA = 3;
int encoder0PinB = 5;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
int startTime, endTime;

DualVNH5019MotorShield md;

void motorLISRA()      //ISR for left motor encoder interrupt
{
    encoder0Pos++;
}

void setup() {
  md.init();
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  Serial.begin (9600);
  startTime = millis();
  endTime = startTime;

  PCintPort::attachInterrupt(encoder0PinA, &motorLISRA, RISING);
}

void loop() {
  endTime = millis();
//  if (endTime - startTime <= 1000) {
    md.setSpeeds(200,185);
    n = digitalRead(encoder0PinA);
//    if ((encoder0PinALast == LOW) && (n == HIGH)) {
//      if (digitalRead(encoder0PinB) == LOW) {
//        encoder0Pos--;
//      } else {
//        encoder0Pos++;
//      }
      Serial.print(endTime);
      Serial.print(" ");
      Serial.println(encoder0Pos);
//    }
    encoder0PinALast = n;
//  }
}


