#include <BasicLinearAlgebra.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <DoubleClicker.h>

using namespace BLA;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo0;
//MPU6050 mpu(Wire);
MPU6050 mpu2(Wire);
DoubleClicker doubleClicker(7,500,300,0);

/*
void getTranslation(BLA::Matrix<3,3> mat, double* vector) {
  vector[0] = mat(0,3);
  vector[1] = mat(1,3);
  vector[2] = mat(2,3);
}
*/
/*
void getTranslationV(BLA::Matrix<4,1> v, double* vector) {
  vector[0] = v(0,0);
  vector[1] = v(1,0);
  vector[2] = v(2,0);
}
*/
/*
void crossProduct(BLA::Matrix<4,1> v1, BLA::Matrix<4,1> v2, double* c) {
  c[0] = v1(1,0) * v2(2,0) - v1(2,0) * v2(1,0);
  c[1] = v1(2,0) * v2(0,0) - v1(0,0) * v2(2,0);
  c[2] = v1(0,0) * v2(1,0) - v1(1,0) * v2(0,0);
}
*/
/*
double deadzoneValue(double value, double min, double dmin, double dmax, double max) {
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  if (value > dmin && value < dmax) {
    return 0;
  }
  if (value >= dmax && value <= max) {
    return max * ((value - dmax) / (max - dmax));
  }
  if (value <= dmin && value >= min) {
    return min * ((dmin - value) / (dmin - min));
  }
  return value;
} 
*/
/*
void newAngleValue(double* oldAbsValue, double newAbsValue, double maxAbs, double t, double* targetValue, double* currentValue) {
  double newCurrentValue = *currentValue;
  double dV = newAbsValue - *oldAbsValue;
  *oldAbsValue = newAbsValue;
  newCurrentValue += dV;
  if (newCurrentValue > maxAbs) {
    newCurrentValue = maxAbs;
  }
  if (newCurrentValue < -maxAbs) {
    newCurrentValue = -maxAbs;
  }
  *currentValue = newCurrentValue;
  *targetValue = t * (*targetValue) + (1-t) * newCurrentValue;
  //*currentValue = (1-t) * (*targetValue) + t * (*currentValue);
} 
*/

enum EEulerOrder
{
    ORDER_XYZ,
    ORDER_YZX,
    ORDER_ZXY,
    ORDER_ZYX,
    ORDER_YXZ,
    ORDER_XZY
};

enum HandState
{
    NONE,
    FOOD_START,
    FOOD_MAIN,
    FOOD_HOLD,
    FOOD_START_HOLD
};

double Sx,Sy,Sz,Cx,Cy,Cz;
BLA::Matrix<3,3> eulerAnglesToMatrix(double x, double y, double z, EEulerOrder EulerOrder)
{
    BLA::Matrix<3,3> Mx;

    Sx = sin(x * PI / 180 );
    Sy = sin(y * PI / 180 );
    Sz = sin(z * PI / 180 );
    Cx = cos(x * PI / 180 );
    Cy = cos(y * PI / 180 );
    Cz = cos(z * PI / 180 );

    switch(EulerOrder)
    {
    case ORDER_XYZ:
        Mx=
        {Cy*Cz,
        -Cy*Sz,
        Sy,
        Cz*Sx*Sy+Cx*Sz,
        Cx*Cz-Sx*Sy*Sz,
        -Cy*Sx,
        -Cx*Cz*Sy+Sx*Sz,
        Cz*Sx+Cx*Sy*Sz,
        Cx*Cy
        };
        break;

    case ORDER_YZX:
        Mx=
        {Cy*Cz,
        Sx*Sy-Cx*Cy*Sz,
        Cx*Sy+Cy*Sx*Sz,
        Sz,
        Cx*Cz,
        -Cz*Sx,
        -Cz*Sy,
        Cy*Sx+Cx*Sy*Sz,
        Cx*Cy-Sx*Sy*Sz
        };
        break;

    case ORDER_ZXY:
        Mx=
        {Cy*Cz-Sx*Sy*Sz,
        -Cx*Sz,
        Cz*Sy+Cy*Sx*Sz,
        Cz*Sx*Sy+Cy*Sz,
        Cx*Cz,
        -Cy*Cz*Sx+Sy*Sz,
        -Cx*Sy,
        Sx,
        Cx*Cy
        };
        break;

    case ORDER_ZYX:
        Mx=
        {Cy*Cz,
        Cz*Sx*Sy-Cx*Sz,
        Cx*Cz*Sy+Sx*Sz,
        Cy*Sz,
        Cx*Cz+Sx*Sy*Sz,
        -Cz*Sx+Cx*Sy*Sz,
        -Sy,
        Cy*Sx,
        Cx*Cy
        };
        break;

    case ORDER_YXZ:
        Mx=
        {Cy*Cz+Sx*Sy*Sz,
        Cz*Sx*Sy-Cy*Sz,
        Cx*Sy,
        Cx*Sz,
        Cx*Cz,
        -Sx,
        -Cz*Sy+Cy*Sx*Sz,
        Cy*Cz*Sx+Sy*Sz,
        Cx*Cy
        };
        break;

    case ORDER_XZY:
        Mx=
        {Cy*Cz,
        -Sz,
        Cz*Sy,
        Sx*Sy+Cx*Cy*Sz,
        Cx*Cz,
        -Cy*Sx+Cx*Sy*Sz,
        -Cx*Sy+Cy*Sx*Sz,
        Cz*Sx,
        Cx*Cy+Sx*Sy*Sz
        };
        break;
    }
    return Mx;
}

double p1,p2,p3,Dsc,perpAngle;
void angleToSurface(double A, double B, double C, double D, BLA::Matrix<3,3> r, double a1, double a2, double* angles) {
  p1 = a2*(A*(-r(0,0)) + B*(-r(1,0)) + C*(-r(2,0)));
  p2 = a2*(A*r(0,1) + B*r(1,1) + C*r(2,1));
  p3 = a1*(A*r(0,1) + B*r(1,1) + C*r(2,1)) + D;
  Dsc = 4*(p1*p1 + p2*p2 - p3*p3);
  
  if (Dsc == 0) {
    angles[0] = 2;
    angles[1] = 2*atan((-p1)/(p3-p2)) * (180 / PI);
    angles[2] = 9999999;
    return;
  }
  if (Dsc > 0) {
    angles[0] = 2;
    angles[1] = 2*atan((-2*p1+sqrt(Dsc))/(2*(p3-p2))) * (180 / PI);
    angles[2] = 2*atan((-2*p1-sqrt(Dsc))/(2*(p3-p2))) * (180 / PI);
    return;
  }
  if (A*r(0,1) + B*r(1,1) + C*r(2,1) == 0) {
    angles[0] = 2;
    angles[1] = 90;
    angles[2] = 9999999;
    return;
  }
  angles[0] = 2;
  perpAngle = atan(-(A*r(0,0) + B*r(1,0) + C*r(2,0))/(A*r(0,1) + B*r(1,1) + C*r(2,1))) * (180 / PI);
  angles[1] = perpAngle;
  if (perpAngle < 0) {
    angles[2] = perpAngle + 180;
  } else {
    angles[2] = perpAngle - 180;
  }
}

bool isAngleInValidRange(double angle){
  return angle >= 0 && angle <= 160;
}

double delta1, delta2;
double furthestOfTwoAngles(double angle1, double angle2) {
  delta1 = abs(angle1 - 0);
  delta2 = abs(angle2 - 0);
  if (delta1 < delta2) {
    return angle1;
  } else {
    return angle2;
  }
}

double sign(double value) {
  if (value < 0) {
    return -1;
  }
  return 1;
}

double clampValue(double value, double min, double max) {
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
} 

double scaleAngle(double angle) {
  return 190 * angle / 180;
}

double clampAngle(double angle) {
  if (angle < 10) {
    return 10;
  }
  if (angle > 170) {
    return 170;
  }
  return angle;
}

double vectorLen(double* vector) {
  return sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2]);
}

double vectorLen(BLA::Matrix<3,1> vector) {
  return sqrt(vector(0,0)*vector(0,0)+vector(1,0)*vector(1,0)+vector(2,0)*vector(2,0));
}

double angleToSurface(BLA::Matrix<3,1> vector, double* normal) {
  return asin((vector(0,0)*normal[0]+vector(1,0)*normal[1]+vector(2,0)*normal[2]) / (vectorLen(vector) * vectorLen(normal))) * 180 / PI;
}

void copyArray(double* original, double* target, int size) {
  for (int i = 0; i < size; i++) {
    target[i] = original[i];
  }
}

double maxComponentDifference(double* arr1, double* arr2, int size) {
  double maxDiff = 0;
  for (int i = 0; i < size; i++) {
    double diff = abs(arr1[i] - arr2[i]);
    if (diff > maxDiff) {
      maxDiff = diff;
    }
  }
  return maxDiff;
}

void smoothArray(double* newArray, double* previous, int size, double t) {
  for (int i = 0; i < size; i++) {
  //  if (i == 2 && abs(newArray[i] - previous[i]) > 90) {
  //    newArray[i] = previous[i];
  //  } else {
      newArray[i] = newArray[i] * (1-t) + t * previous[i];
  //  }
  }
}

//BLA::Matrix<4,4> palmForward = {1,0,0,0,
//                                0,1,0,1,
//                                0,0,1,0,
//                                0,0,0,1};
//BLA::Matrix<4,4> palmUp = {1,0,0,0,
//                            0,1,0,0,
//                            0,0,1,1,
//                            0,0,0,1};
//BLA::Matrix<4,4> rVH;

BLA::Matrix<3,1> vForward = {0,1,0};
BLA::Matrix<3,1> vUp = {0,0,1};
double side[3] = {0,0,0};
BLA::Matrix<3,1> vPalm = {0,0,0};
BLA::Matrix<3,1> palmUp = {0,0,0};
double beta = 0;
double alpha = 0;
double lenMultiplier = 0;
double s = 0;
void palmMotorAngles( BLA::Matrix<3,3> palmRotation, double vX, double vZ, double* angles) {                              
  vPalm = Inverse(palmRotation) * eulerAnglesToMatrix(vX, 0, vZ, EEulerOrder::ORDER_ZYX) * vForward;
  if (vPalm(1,0) < 0) {
    vPalm(1,0) = 0;
    lenMultiplier = 1 / sqrt(vPalm(0,0) * vPalm(0,0) + vPalm(2,0) * vPalm(2,0));
    vPalm(0,0) = vPalm(0,0) * lenMultiplier;
    vPalm(2,0) = vPalm(2,0) * lenMultiplier;
  } 
  beta = asin(vPalm(2,0)); 
  if (cos(beta) != 0) {
    s = vPalm(0,0) / cos(beta);
    if (s > 1) {
      s = 1;
    }
    if (s < -1) {
      s = -1;
    }
    alpha = -asin(s);
  }        
  angles[0] = beta * 180 / PI; 
  angles[1] = alpha * 180 / PI;

  side[0] = cos(vZ / 180 * PI); side[1] = sin(vZ / 180 * PI);
  palmUp = palmRotation * eulerAnglesToMatrix(angles[0], 0, angles[1], EEulerOrder::ORDER_ZXY) * vUp;
  angles[2] = angleToSurface(palmUp, side);
  if (palmUp(2,0) < 0) {
    angles[2] = 180 - angles[2];
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(7, INPUT_PULLUP);
  //mpu.setAddress(0x68);
  mpu2.setAddress(0x69);
  Wire.begin();
  //byte status = mpu.begin(1,0,1);
  byte status2 = mpu2.begin(1,0,3);
  while(/*status!=0 &&*/ status2!=0){Serial.println("No signal from MPUs");}
  //mpu.calcOffsets();
  mpu2.calcOffsets();
  Serial.println("STARTED!\n");
  servo1.attach(9);
  servo1.write(90);  //90
  servo2.attach(6);
  servo2.write(90);  //90
  servo3.attach(5);
  servo3.write(90);  //90
  servo4.attach(10);
  servo4.write(0);
  servo0.attach(11);
  servo0.write(0);
}

BLA::Matrix<3,1> mpuHeadJointVec = {0,0,-1};
BLA::Matrix<3,3> mpuHandRot4;
BLA::Matrix<3,3> mpuHeadRot4;
BLA::Matrix<3,3> armRot4;
BLA::Matrix<3,3> palmRot4;

int currentAngle = 0;
int prevCurrentAngle = 0;
int state = 0;

bool zCalibrated = false;
double zOffset = 0;
double prevZ = 0;
double dZ = 0;
double currentZ = 0;
double currentX = 0;
double originX = 0;
double vX = 0;
double vZ = 0;
double currentY = 0;
double vY = 0;

double palmAngles[3] = {0,0,0};
double armAngles[3] = {0,0,0};

const double h = 0.12;
const double a1 = 0.34;
const double a2 = 0.34;

int buttonState = 1;
int buttonClamp = 1;

double newAngle = 99999999;
double angle1;
double angle2;

const double headPalmThreshold = 10;
const double headPalmThresholdY = 10;
const double palmThreshold = 90;

bool buttonJustPressed = true;
bool buttonJustReleased = true;

double prevAngles[3] = {0,0,0};

//Smoothing
double t = 0.9;

int calibrationFrames = 20;
double zSum = 0;
int calibrationCount = 0;

HandState handState = FOOD_START;
BLA::Matrix<3,1> arm = {0,0,0};
BLA::Matrix<3,1> palmPosition = {0,0,0};
double distanceToHead = 0;

double downAngle = 10;
double rollOffset = -10;
double mult = 0;

unsigned long holdExitTime = 0;

void loop() {
  doubleClicker.update();
  mpu2.update();
  mpuHandRot4 = eulerAnglesToMatrix(0,90 + mpu2.getAngleY(),mpu2.getAngleX(), EEulerOrder::ORDER_YXZ);
  angleToSurface(mpuHeadJointVec(0,0), mpuHeadJointVec(1,0), mpuHeadJointVec(2,0), -mpuHeadJointVec(2,0) * h, mpuHandRot4, a1, a2, armAngles);
  newAngle = 99999999;
  angle1 = armAngles[1];
  angle2 = armAngles[2];
  if (isAngleInValidRange(angle1) && isAngleInValidRange(angle2)) {
    newAngle = furthestOfTwoAngles(angle1, angle2);
  } else {
    if (isAngleInValidRange(angle1)) {
      newAngle = angle1;
    }
    if (isAngleInValidRange(angle2)) {
      newAngle = angle2;
    }
  }
  if (!isAngleInValidRange(newAngle) || (abs(currentAngle - newAngle) > 45)) {
    newAngle = currentAngle;
  }
  currentAngle = newAngle;

  //State machine
  switch (handState) {
    case FOOD_START:
      armRot4 = eulerAnglesToMatrix(0,  0, currentAngle, EEulerOrder::ORDER_ZXY);
      palmRot4 = mpuHandRot4 * armRot4;
      vX = -downAngle;
      vZ = 0 + 45;
      copyArray(palmAngles, prevAngles, 3);
      palmMotorAngles( palmRot4, vX, vZ, palmAngles);
      smoothArray(palmAngles, prevAngles, 3, t);
      servo2.write(clampAngle(palmAngles[0] + 90));
      servo1.write(clampAngle(palmAngles[1] + 90));
      servo3.write(clampAngle(-palmAngles[2] + 90 + rollOffset)); //Roll
      servo0.write(clampAngle(currentAngle));
      servo4.write(60);
      if (doubleClicker.getState() == 1) {
        handState = FOOD_MAIN;
        servo4.write(30);
      }
      if (doubleClicker.getState() == 3) {
        handState = FOOD_START_HOLD;
        prevCurrentAngle = currentAngle;
        copyArray(palmAngles, prevAngles, 3);
        break;
      }
      break;
    case FOOD_MAIN:
      armRot4 = eulerAnglesToMatrix(0,  0, currentAngle, EEulerOrder::ORDER_ZXY);
      palmRot4 = mpuHandRot4 * armRot4;
      arm = {0,a1,0};
      palmPosition = mpuHandRot4 * arm;
      arm = {0,a2,0};
      palmPosition = palmPosition + (mpuHandRot4 * armRot4 * arm);
      distanceToHead = vectorLen(palmPosition);
      if (distanceToHead < 0.4) {
        mult = (0.4 - distanceToHead) / 0.05;
        if (mult > 1) {
          mult = 1;
        }
        vX =  - downAngle;
        vZ = 0 + 45 + mult * 90;
      } else {
        vX =  - downAngle;
        vZ = 0 + 45;
      }
      copyArray(palmAngles, prevAngles, 3);
      palmMotorAngles( palmRot4, vX, vZ, palmAngles);
      if (millis() - holdExitTime > 500) {
        holdExitTime = millis();
        t = 0.9;
      }
      smoothArray(palmAngles, prevAngles, 3, t);
      servo2.write(clampAngle(palmAngles[0] + 90));
      servo1.write(clampAngle(palmAngles[1] + 90));
      servo3.write(clampAngle(-palmAngles[2] + 90 + rollOffset)); //Roll
      servo0.write(clampAngle(currentAngle));
      if (doubleClicker.getState() == 3) {
        handState = FOOD_HOLD;
        prevCurrentAngle = currentAngle;
        copyArray(palmAngles, prevAngles, 3);
        break;
      }
      if (doubleClicker.getState() == 2) {
        handState = FOOD_START;
        servo4.write(60);
      }
      break;
    case FOOD_HOLD:
      armRot4 = eulerAnglesToMatrix(0,  0, prevCurrentAngle, EEulerOrder::ORDER_ZXY);
      arm = {0,a1,0};
      palmPosition = mpuHandRot4 * arm;
      arm = {0,a2,0};
      palmPosition = palmPosition + (mpuHandRot4 * armRot4 * arm);
      servo2.write(prevAngles[0] + 90);
      servo1.write(prevAngles[1] + 90);
      servo3.write(-prevAngles[2] + 90 + rollOffset); //Roll
      servo0.write(clampAngle(prevCurrentAngle));
      if (doubleClicker.getState() == 0) {
        handState = FOOD_MAIN;
        copyArray(prevAngles, palmAngles, 3);
        t = 0.98;
        mpuHeadJointVec = {0, palmPosition(2,0) - h, -palmPosition(1,0)};
        currentAngle = prevCurrentAngle;
        holdExitTime = millis();
      }
      break;
    case FOOD_START_HOLD:
      armRot4 = eulerAnglesToMatrix(0,  0, prevCurrentAngle, EEulerOrder::ORDER_ZXY);
      palmRot4 = mpuHandRot4 * armRot4;
      palmPosition = mpuHandRot4 * arm;
      arm = {0,a2,0};
      palmPosition = palmPosition + (mpuHandRot4 * armRot4 * arm);
      vX = -downAngle;
      vZ = 0 + 45;
      copyArray(palmAngles, prevAngles, 3);
      palmMotorAngles(palmRot4, vX, vZ, palmAngles);
      smoothArray(palmAngles, prevAngles, 3, t);
      servo2.write(clampAngle(palmAngles[0] + 90));
      servo1.write(clampAngle(palmAngles[1] + 90));
      servo3.write(clampAngle(-palmAngles[2] + 90 + rollOffset)); //Roll
      servo0.write(clampAngle(prevCurrentAngle));
      if (doubleClicker.getState() == 0) {
        handState = FOOD_START;
        mpuHeadJointVec = {0, palmPosition(2,0) - h, -palmPosition(1,0)};
        currentAngle = prevCurrentAngle;
      }
      break;
  }
  delay(10);
}

