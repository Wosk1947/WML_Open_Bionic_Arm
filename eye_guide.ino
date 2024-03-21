#include <BasicLinearAlgebra.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>

using namespace BLA;

Servo servo1;
Servo servo2;
Servo servo3;
MPU6050 mpu(Wire);
MPU6050 mpu2(Wire);

enum EEulerOrder
{
    ORDER_XYZ,
    ORDER_YZX,
    ORDER_ZXY,
    ORDER_ZYX,
    ORDER_YXZ,
    ORDER_XZY
};

double Sx,Sy,Sz,Cx,Cy,Cz;
BLA::Matrix<4,4> eulerAnglesToMatrix(double x, double y, double z, EEulerOrder EulerOrder)
{
    BLA::Matrix<4,4> Mx;

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
        0,
        Cz*Sx*Sy+Cx*Sz,
        Cx*Cz-Sx*Sy*Sz,
        -Cy*Sx,
        0,
        -Cx*Cz*Sy+Sx*Sz,
        Cz*Sx+Cx*Sy*Sz,
        Cx*Cy,
        0,
        0,
        0,
        0,
        1};
        break;

    case ORDER_YZX:
        Mx=
        {Cy*Cz,
        Sx*Sy-Cx*Cy*Sz,
        Cx*Sy+Cy*Sx*Sz,
        0,
        Sz,
        Cx*Cz,
        -Cz*Sx,
        0,
        -Cz*Sy,
        Cy*Sx+Cx*Sy*Sz,
        Cx*Cy-Sx*Sy*Sz,
        0,
        0,
        0,
        0,
        1};
        break;

    case ORDER_ZXY:
        Mx=
        {Cy*Cz-Sx*Sy*Sz,
        -Cx*Sz,
        Cz*Sy+Cy*Sx*Sz,
        0,
        Cz*Sx*Sy+Cy*Sz,
        Cx*Cz,
        -Cy*Cz*Sx+Sy*Sz,
        0,
        -Cx*Sy,
        Sx,
        Cx*Cy,
        0,
        0,
        0,
        0,
        1};
        break;

    case ORDER_ZYX:
        Mx=
        {Cy*Cz,
        Cz*Sx*Sy-Cx*Sz,
        Cx*Cz*Sy+Sx*Sz,
        0,
        Cy*Sz,
        Cx*Cz+Sx*Sy*Sz,
        -Cz*Sx+Cx*Sy*Sz,
        0,
        -Sy,
        Cy*Sx,
        Cx*Cy,
        0,
        0,
        0,
        0,
        1};
        break;

    case ORDER_YXZ:
        Mx=
        {Cy*Cz+Sx*Sy*Sz,
        Cz*Sx*Sy-Cy*Sz,
        Cx*Sy,
        0,
        Cx*Sz,
        Cx*Cz,
        -Sx,
        0,
        -Cz*Sy+Cy*Sx*Sz,
        Cy*Cz*Sx+Sy*Sz,
        Cx*Cy,
        0,
        0,
        0,
        0,
        1};
        break;

    case ORDER_XZY:
        Mx=
        {Cy*Cz,
        -Sz,
        Cz*Sy,
        0,
        Sx*Sy+Cx*Cy*Sz,
        Cx*Cz,
        -Cy*Sx+Cx*Sy*Sz,
        0,
        -Cx*Sy+Cy*Sx*Sz,
        Cz*Sx,
        Cx*Cy+Sx*Sy*Sz,
        0,
        0,
        0,
        0,
        1};
        break;
    }
    return Mx;
}

void getTranslation(BLA::Matrix<4,4> mat, double* vector) {
  vector[0] = mat(0,3);
  vector[1] = mat(1,3);
  vector[2] = mat(2,3);
}

double p1,p2,p3,Dsc,perpAngle;
void angleToSurface(double A, double B, double C, double D, BLA::Matrix<4,4> r, double a1, double a2, double* angles) {
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

double furthestOfTwoAngles(double angle1, double angle2) {
  double delta1 = abs(angle1 - 0);
  double delta2 = abs(angle2 - 0);
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

BLA::Matrix<4,4> palmForward = {1,0,0,0,
                                0,1,0,1,
                                0,0,1,0,
                                0,0,0,1};
double vPalm[3] = {0,0,0};
BLA::Matrix<4,4> rVH;
double beta = 0;
double alpha = 0;
void palmMotorAngles(BLA::Matrix<4,4> headRotation, BLA::Matrix<4,4> armRotation, double vX, double vZ, double* angles) {
  rVH = headRotation * eulerAnglesToMatrix(vX, 0, vZ, EEulerOrder::ORDER_ZYX);                              
  getTranslation(Inverse(armRotation) * rVH * palmForward, vPalm);
  if (vPalm[1] < 0) {
    vPalm[1] = 0;
    double lenMultiplier = 1 / sqrt(vPalm[0] * vPalm[0] + vPalm[2] * vPalm[2]);
    vPalm[0] = vPalm[0] * lenMultiplier;
    vPalm[2] = vPalm[2] * lenMultiplier;
  } 
  beta = asin(vPalm[2]); 
  if (cos(beta) != 0) {
    double s = vPalm[0] / cos(beta);
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
}  

void setup() {
  Serial.begin(9600);
  pinMode(7, INPUT_PULLUP);
  mpu.setAddress(0x68);
  mpu2.setAddress(0x69);
  Wire.begin();
  byte status = mpu.begin(1,0,1);
  byte status2 = mpu2.begin(1,0,3);
  while(status!=0 && status2!=0){Serial.println("No signal from MPUs");}
  mpu.calcOffsets();
  mpu2.calcOffsets();
  Serial.println("STARTED!\n");
  //timer = millis();
  servo1.attach(5);
  servo1.write(0);
  servo2.attach(6);
  servo2.write(90);
  servo3.attach(9);
  servo3.write(90);
}

bool calibrated = false;
BLA::Matrix<4,4> calMat1 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
BLA::Matrix<4,4> calMat2 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
BLA::Matrix<4,4> mpuHeadUp = {1,0,0,0,
                                0,1,0,0,
                                0,0,1,1,
                                0,0,0,1};
double mpuHeadJointVec[3] = {0,0,0};
BLA::Matrix<4,4> mpuHandRot4;
BLA::Matrix<4,4> mpuHeadRot4;
BLA::Matrix<4,4> armRot4;

int currentAngle = 0;
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

double palmAngles[2] = {0,0};
double armAngles[3] = {0,0,0};

const double h = 0.22;
const double a1 = 0.30;
const double a2 = 0.31;

int buttonState = 1;

double newAngle = 99999999;
double angle1;
double angle2;

const double headPalmThreshold = 8;
const double palmThreshold = 90;

bool buttonJustPressed = true;
bool buttonJustReleased = true;

void loop() {
  buttonState = digitalRead(7);
  if (buttonState == 1) {
    servo2.write(90);
    servo3.write(90);
    buttonJustPressed = true;
    mpu.update();
    mpuHeadRot4 = eulerAnglesToMatrix(mpu.getAngleX(), mpu.getAngleY(), 0, EEulerOrder::ORDER_YXZ);
    mpu2.update();
    mpuHandRot4 = eulerAnglesToMatrix(0,90 + mpu2.getAngleY(),mpu2.getAngleX(), EEulerOrder::ORDER_YXZ);
    getTranslation(mpuHeadRot4 * mpuHeadUp, mpuHeadJointVec);
    angleToSurface(mpuHeadJointVec[0], mpuHeadJointVec[1], mpuHeadJointVec[2], -mpuHeadJointVec[2] * h, mpuHandRot4, a1, a2, armAngles);
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
    if (!isAngleInValidRange(newAngle) || (abs(currentAngle - newAngle) > 45 && !buttonJustReleased)) {
      newAngle = currentAngle;
    }
    currentAngle = newAngle;
    servo1.write(currentAngle);
    if (buttonJustReleased) {
      buttonJustReleased = false;
    }
  } else if (buttonState == 0){
    if (buttonJustPressed) {
      buttonJustPressed = false;
      buttonJustReleased = true;
      armRot4 = mpuHandRot4 * eulerAnglesToMatrix(0, 0, currentAngle, EEulerOrder::ORDER_YXZ);
      mpuHeadRot4 = eulerAnglesToMatrix(mpu.getAngleX(), mpu.getAngleY(), -15, EEulerOrder::ORDER_YXZ);
      originX = mpu.getAngleX();
      currentZ = 0;
    }
    mpu.update();
    if (!zCalibrated) {
      zCalibrated = true;
      zOffset = mpu.getAngleZ();
      prevZ = zOffset;
    }
    dZ = mpu.getAngleZ() - prevZ;
    if (abs(dZ) < 0.08) {dZ = 0;}
    prevZ = mpu.getAngleZ();
    currentZ += dZ;
    currentX = mpu.getAngleX() - originX;
    if (currentX < -headPalmThreshold) {
      currentX = -headPalmThreshold;
    }
    if (currentX > headPalmThreshold) {
      currentX = headPalmThreshold;
    }
    if (currentZ < -headPalmThreshold || currentZ > headPalmThreshold) {
      currentZ -= dZ;
    }
    vX = palmThreshold * currentX / headPalmThreshold;
    vZ = palmThreshold * currentZ / headPalmThreshold;

    palmMotorAngles(mpuHeadRot4, armRot4, vX, vZ, palmAngles);
    servo2.write(palmAngles[1] + 90);
    servo3.write(palmAngles[0] + 90);
  }
  delay(10);
}

