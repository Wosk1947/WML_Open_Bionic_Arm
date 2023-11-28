#include "Arduino.h"
#include "ServoVoltageNon360.h"

    ServoVoltageNon360::ServoVoltageNon360(int fPin, int bPin, int sPin, int pPin, int min, int max) {
      forwardPin = fPin;
      backwardPin = bPin;
      speedPin = sPin;
      potPin = pPin;
      minAngle = min;
      maxAngle = max;

      Serial.begin(9600);
      pinMode(forwardPin, OUTPUT);
      pinMode(backwardPin, OUTPUT);
      pinMode(potPin, INPUT);
    }

    void ServoVoltageNon360::setTargetAngle(int angle){
      if (angle < minAngle || angle > maxAngle) {
        return;
      }
      targetAngle = angle;
    }

    void ServoVoltageNon360::update() {
      int targetAngleInternal = potentiometerFromAngle(targetAngle);
      int pot = analogRead(potPin);
      currentAngle = pot;
      currentSpeed = getSpeed(currentAngle, targetAngleInternal, currentDirection);
      currentDirection = getDirection(currentAngle, targetAngleInternal);
      move();
    }

    void ServoVoltageNon360::move() {
      if (currentDirection == 1) {
        analogWrite(speedPin, currentSpeed);
        digitalWrite(forwardPin, HIGH);
        digitalWrite(backwardPin, LOW);
      } else if(currentDirection == -1) {
        analogWrite(speedPin, currentSpeed);
        digitalWrite(forwardPin, LOW);
        digitalWrite(backwardPin, HIGH);
      }
    }

    int ServoVoltageNon360::getSpeed(int cAngle, int angle, int direction) {
      float multiplier = 1;
      float sMin = 55;
      float sMax = 100;
      float dSpeed = maxSpeed - sMin - sMax;
      int t = 70;
      if (abs(cAngle - angle) < t) {
        multiplier = abs(cAngle - angle) / t;
      }
      if (cAngle > angle && direction == -1) {
        //Stop condition
      }
      if (cAngle < angle && direction == 1) {
        //Stop condition
      }
      return (int)(sMin + dSpeed * multiplier);
    }

    int ServoVoltageNon360::getDirection(int cAngle, int angle) {
      if (cAngle < angle) {
        return -1;
      } else {
        return 1;
      }
    }

    int ServoVoltageNon360::getTargetAngle() {
      return targetAngle;
    }
    int ServoVoltageNon360::getCurrentAngle() {
      return angleFromPotentiometer(currentAngle);
    }
    int ServoVoltageNon360::getCurrentRawAngle() {
      return currentAngle;
    }
    int ServoVoltageNon360::angleFromPotentiometer(int pot) {
      return 0.2316 * pot - 38.227;
    }
    int ServoVoltageNon360::potentiometerFromAngle(int angle) {
      return (angle + 38.277) / 0.2316;
    }
