#ifndef ServoVoltageNon360_h
#define ServoVoltageNon360_h
#include "Arduino.h" 
class ServoVoltageNon360 {
public:
  int forwardPin;
  int backwardPin;
  int speedPin;
  int potPin;
  int minAngle;
  int maxAngle;

  ServoVoltageNon360(int fPin, int bPin, int sPin, int pPin, int min, int max);
  void setTargetAngle(int angle);
  void update();
  int getTargetAngle();
  int getCurrentAngle();
  int getCurrentRawAngle();
private:
  int currentDirection;
  int currentAngle;
  int currentSpeed;
  const int maxSpeed = 255;
  int targetAngle = 0;

  void move();
  int getSpeed(int cAngle, int angle, int direction);
  int getDirection(int cAngle, int angle);
  int angleFromPotentiometer(int pot);
  int potentiometerFromAngle(int angle);
};
#endif