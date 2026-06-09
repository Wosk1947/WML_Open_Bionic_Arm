#include <BasicLinearAlgebra.h>
#include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <EMGFilters.h>

#define TIMING_DEBUG 1
#define SensorInputPin A0 // input pin number

using namespace BLA;

Servo palm;
Servo elbow;
MPU6050 mpu(Wire);
EMGFilters myFilter;

//EMG Vars
int sampleRate = SAMPLE_FREQ_1000HZ;
int humFreq = NOTCH_FREQ_50HZ;

const int bufferSize = 50; 
double rawBuffer[bufferSize] = {0};
double smthBuffer[bufferSize] = {0};
double clampedSmthBuffer[bufferSize] = {0}; 
double modBuffer[bufferSize] = {0};
double spikeBuffer[bufferSize] = {0};
double spikeEndBuffer[bufferSize] = {0};

double steepnessDown = 0.4;
double steepnessUp = 0.2;
double delta = 0.5;
double sensitivity(double value, double span, double steepness) {
  if (value > span) {
    return 1;
  }
  if (value < 0) {
    return 0;
  }
  return (exp(steepness * value) - 1) / (exp(steepness * span) - 1);
}

void printArray(double arr[], int size) {
  Serial.print("[ ");
  for (int i = 0; i < size - 1; i++) {
    Serial.print(String(arr[i]) + " , ");
  }
  Serial.print(String(arr[size - 1]) + " ]");
  Serial.println();
}

void shiftBuffer(double buffer[], int size, int n) {
  for (int i = 0; i < size - n; i++) {
    buffer[i] = buffer[i + n];
  }
  for (int i = max(size - n, 0); i < size; i++) {
    buffer[i] = 0;
  }
}

void addToBuffer(double buffer[], int size, double value) {
  shiftBuffer(buffer, size, 1);
  buffer[size - 1] = value;
}

void clearBuffer(double buffer[], int size) {
  for (int i = 0; i < size; i++) {
    buffer[i] = 0;
  }
}

double maxInBuffer(double buffer[], int size) {
  double res = 0;
  for (int i = 0; i < size; i++) {
    res = max(res, buffer[i]);
  }
}

void setRangeOfBufferToValue(double buffer[], int size, int start, int end, double value) {
  for (int i = max(0, start); i < min(size, end); i++) {
    buffer[i] = value;
  }
}

double distance(double a, double b) {
  return abs(a - b);
}

double initialSteepness = 30;
boolean checkStartOfSpike(double buffer[], int size) {
  return (buffer[size - 1] - buffer[0] > initialSteepness);
}

void copyBuffers(double origin[], double target[], int size) {
  for (int i = 0; i < size; i++) {
    target[i] = origin[i];
  }
}

//EMA
double alpha = 0.999; //0.999
double prevSmthVal = 0;
double currSmthVal = 0;
double clampedPrevSmthVal = 0;
double clampedCurrSmthVal = 0;
//Controller
double centerVal = 50;
double maxVal = 53;
boolean readyForSpike = true;
boolean spike = false;
boolean endSpike = false;

int sosPeriod = 50;
double thresholdVal = 1400; //1500
boolean checkStartOfSpike2(double buffer[], int size) {
  double sum = 0;
  for (int i = size - 1; i >= max(0, size - sosPeriod); i--) {
    sum += buffer[i];
  }
  sum /= min(size, sosPeriod);
  return sum >= thresholdVal;
}

int period = 50;
boolean checkEndOfSpike(double buffer[], int size) {
  double sum = 0;
  for (int i = size - 1; i >= max(0, size - period); i--) {
    sum += buffer[i];
  }
  sum /= min(size, period);
  return sum < centerVal;
  //return buffer[size - 1] < buffer[size - 2];
}

boolean checkOverflowEnd(double buffer[], int size) {
  double sum = 0;
  for (int i = size - 1; i >= max(0, size - period); i--) {
    sum += buffer[i];
  }
  sum /= min(size, period);
  return sum < maxVal;
}

void updateController(double rawVal) {
  endSpike = false;
  addToBuffer(rawBuffer, bufferSize, rawVal);
  //EMA
  currSmthVal = prevSmthVal * alpha + (1-alpha) * rawVal;
  prevSmthVal = currSmthVal;
  addToBuffer(smthBuffer, bufferSize, currSmthVal);

  //ClampedEMA
  if (currSmthVal <= maxVal) {
    addToBuffer(clampedSmthBuffer, bufferSize, currSmthVal);
  } else {
    addToBuffer(clampedSmthBuffer, bufferSize, maxVal);
    if (checkOverflowEnd(rawBuffer, bufferSize)) {
        prevSmthVal = maxVal;
    }
  }

  //if (readyForSpike && /*checkStartOfSpike(smthBuffer, bufferSize)*/ checkStartOfSpike2(rawBuffer, bufferSize)) {
  //  spike = true;
  //  readyForSpike = false;
  //  setRangeOfBufferToValue(modBuffer, bufferSize, 0, bufferSize, centerVal);
  //}
  //if (spike) {
  //  addToBuffer(modBuffer, bufferSize, centerVal);
  //  if (checkEndOfSpike(rawBuffer, bufferSize)) {
  //    endSpike = true;
  //    spike = false;
  //    readyForSpike = true;
  //    prevSmthVal = centerVal;
  //    copyBuffers(modBuffer, smthBuffer, bufferSize);
  //  }
  //} else {
  //  addToBuffer(modBuffer, bufferSize, clampedSmthBuffer[bufferSize - 1]); //currSmthVal
  //}
  
}

double clampPalmAngle(double angle) {
  return max(min(angle, 80), 10);
}

//PalmController
double currentPalmAngle = 120;
double deadzone = 10;
void updatePalmController(double val) {
  if (val > centerVal) {
    currentPalmAngle -= 15;
  }
  if (val < centerVal) {
    currentPalmAngle += 60 /** sensitivity(abs(val-centerVal), abs(maxVal - centerVal) * 0.3, steepnessDown)*/;
  }
  //Serial.println(val-centerVal);
  //Serial.println(delta * sensitivity(abs(val-centerVal), maxVal - centerVal));
  currentPalmAngle = clampPalmAngle(currentPalmAngle);
}

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
    START,
    GRAB,
    ADJUST,
    AFTER_ADJUST
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

double scaleAngle(double angle) {
  return 190 * angle / 180;
}

double clampAngle(double angle) {
  if (angle < 10) {
    return 10;
  }
  if (angle > 140) {
    return 140;
  }
  return angle;
}

bool usePalm = true;
bool useElbow = true;

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP");
  pinMode(13, INPUT_PULLUP);
  pinMode(11, OUTPUT);
  mpu.setAddress(0x68);
  Wire.begin();
  byte status = mpu.begin(1,0,3);
  while(status!=0){Serial.println("No signal from MPUs");}
  mpu.calcOffsets();
  Serial.println("STARTED!\n");
  myFilter.init(sampleRate, humFreq, true, true, true);

  if (usePalm) {
    palm.attach(5);
    palm.write(80);
  }
  if (useElbow) {  
    elbow.attach(6);
    elbow.write(10);
  }
}

BLA::Matrix<3,1> mpuHeadJointVec = {0,0,-1};
BLA::Matrix<3,3> mpuHandRot4;
BLA::Matrix<3,3> armRot4;

int currentAngle = 0;
int prevCurrentAngle = 0;
double currentEMA = 0;

double armAngles[3] = {0,0,0};

const double h = 0.24/*0.12*/;
const double a1 = 0.32/*0.34*/;
const double a2 = 0.4/*0.34*/;

double newAngle = 99999999;
double angle1;
double angle2;

HandState handState = START;
BLA::Matrix<3,1> arm = {0,0,0};
BLA::Matrix<3,1> palmPosition = {0,0,0};

double rawVal = 0;
long adjustExitTime = 0;
long afterAdjustCooldown = 400;

double finalEMGVal = 0;
double hState = 0;

bool holding = false;
bool readyToOpen = false;
bool onOpen = false;

void loop() {
  int button = digitalRead(13);
  if (button == 0) {
    digitalWrite(11, HIGH);
  } else {
    digitalWrite(11, LOW);
  }

  //EMG Filter
  int value = analogRead(SensorInputPin);
  int DataAfterFilter = myFilter.update(value);
  rawVal = sq(DataAfterFilter);
  updateController(rawVal);

  //Gyro
  mpu.update();
  mpuHandRot4 = eulerAnglesToMatrix(0,90 + mpu.getAngleY(), mpu.getAngleX(), EEulerOrder::ORDER_YXZ);
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

  double eT = 0.8;
  currentAngle = currentAngle * eT + (1 - eT) * newAngle;

  //State machine
  switch (handState) {
    case START:
      prevCurrentAngle = currentAngle;
      handState = GRAB;
      break;
    case AFTER_ADJUST:
      //digitalWrite(11, HIGH);
      if (smthBuffer[0] >= maxVal) {
        handState = GRAB;
      }
      if (button == 0) {
        armRot4 = eulerAnglesToMatrix(0,  0, prevCurrentAngle, EEulerOrder::ORDER_ZXY);
        arm = {0,a1,0};
        palmPosition = mpuHandRot4 * arm;
        arm = {0,a2,0};
        palmPosition = palmPosition + (mpuHandRot4 * armRot4 * arm);
        mpuHeadJointVec = {0, palmPosition(2,0) - h, -palmPosition(1,0)};
        currentAngle = prevCurrentAngle;
        handState = ADJUST;
        hState = 100;
      }
      break;
    case GRAB:
      finalEMGVal = smthBuffer[bufferSize - 1];
      if (finalEMGVal > centerVal && !holding) {
        palm.write(10);
        holding = true;
        readyToOpen = false;
        onOpen = false;
      }
      if (holding) {
        if (finalEMGVal < centerVal) {
          readyToOpen = true;
        }
      }
      if (holding && readyToOpen) {
        if (finalEMGVal > centerVal) {
          onOpen = true;
        }
      }
      if (holding && onOpen) {
        if (finalEMGVal < centerVal) {
          palm.write(80);
          holding = false;
          readyToOpen = false;
          onOpen = false;
        }
      }
      if (button == 0) {
        armRot4 = eulerAnglesToMatrix(0,  0, prevCurrentAngle, EEulerOrder::ORDER_ZXY);
        arm = {0,a1,0};
        palmPosition = mpuHandRot4 * arm;
        arm = {0,a2,0};
        palmPosition = palmPosition + (mpuHandRot4 * armRot4 * arm);
        mpuHeadJointVec = {0, palmPosition(2,0) - h, -palmPosition(1,0)};
        currentAngle = prevCurrentAngle;
        handState = ADJUST;
        hState = 100;

        readyToOpen = false;
        onOpen = false;
      }
      break;
    case ADJUST:
    if (useElbow) {
      elbow.write(clampAngle(currentAngle));
    }
      if (button == 1) {
        adjustExitTime = millis();
        prevCurrentAngle = currentAngle;
        handState = GRAB;
        hState = 0;
      }
      break;
  }

  bool logEMG = false;
  if (logEMG) {
    Serial.print(finalEMGVal);
    Serial.print("\t");
    Serial.print(currentAngle);
    Serial.print("\t");
    Serial.print(smthBuffer[bufferSize - 1]);
    Serial.println();
  }
  
  delayMicroseconds(500);
}

