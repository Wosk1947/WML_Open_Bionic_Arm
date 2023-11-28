
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <MadgwickAHRS.h>
#include <ServoVoltageNon360.h>

using namespace BLA;

MPU6050 mpu(Wire);
MPU6050 mpu2(Wire);
unsigned long timer = 0;

ServoVoltageNon360 servo(8,12,11,A1,10,170);


enum EEulerOrder
{
    ORDER_XYZ,
    ORDER_YZX,
    ORDER_ZXY,
    ORDER_ZYX,
    ORDER_YXZ,
    ORDER_XZY
};

BLA::Matrix<4,4> makeMatrix4(BLA::Matrix<3,3> mat3) {
  BLA::Matrix<4,4> Mx = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      Mx(i,j) = mat3(i,j);
    } 
  }
  return Mx;
}

BLA::Matrix<4,4> eulerAnglesToMatrix(BLA::Matrix<3,1> angles, EEulerOrder EulerOrder)
{
    // Convert Euler Angles passed in a vector of Radians
    // into a rotation matrix.  The individual Euler Angles are
    // processed in the order requested.
    BLA::Matrix<3,3> Mx;

    const float    pi    = 3.141592;

    const float    Sx    = sin(angles(0,0) * pi / 180 );
    const float    Sy    = sin(angles(1,0) * pi / 180 );
    const float    Sz    = sin(angles(2,0) * pi / 180 );
    const float    Cx    = cos(angles(0,0) * pi / 180 );
    const float    Cy    = cos(angles(1,0) * pi / 180 );
    const float    Cz    = cos(angles(2,0) * pi / 180 );

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
        Cx*Cy};
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
        Cx*Cy-Sx*Sy*Sz};
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
        Cx*Cy};
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
        Cx*Cy};
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
        Cx*Cy};
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
        Cx*Cy+Sx*Sy*Sz};
        break;
    }
    return makeMatrix4(Mx);
}

BLA::Matrix<4,4> quaternionToMatrix(BLA::Matrix<4,1> quat) {
  double s = quat(0,0);
  double x = quat(1,0);
  double y = quat(2,0);
  double z = quat(3,0);
  BLA::Matrix<3,3> Mx = {
    1-2*y*y-2*z*z, 2*x*y - 2*s*z, 2*x*z + 2*s*y,
    2*x*y + 2*s*z, 1-2*x*x-2*z*z, 2*y*z - 2*s*x,
    2*x*z - 2*s*y, 2*y*z + 2*s*x, 1-2*x*x-2*y*y
  };
  return makeMatrix4(Mx);
}

BLA::Matrix<3,1> getTranslation(BLA::Matrix<4,4> mat) {
  BLA::Matrix<3,1> Mx = {0,0,0};
  Mx(0,0) = mat(0,3);
  Mx(1,0) = mat(1,3);
  Mx(2,0) = mat(2,3);
  return Mx;
}

BLA::Matrix<3,1> angleToSurface(BLA::Matrix<4,1> Surface, BLA::Matrix<4,4> r, double a1, double a2) {
  BLA::Matrix<3,1> angles = {0,0,0};
  double A = Surface(0,0); double B = Surface(1,0); double C = Surface(2,0); double D = Surface(3,0);
  double p1 = a2*(A*(-r(0,0)) + B*(-r(1,0)) + C*(-r(2,0)));
  double p2 = a2*(A*r(0,1) + B*r(1,1) + C*r(2,1));
  double p3 = a1*(A*r(0,1) + B*r(1,1) + C*r(2,1)) + D;
  double Dsc = 4*(p1*p1 + p2*p2 - p3*p3);
  
  if (Dsc == 0) {
    double root = 2*atan((-p1)/(p3-p2));
    angles(0,0) = 2;
    angles(1,0) = root * (180 / PI);
    angles(2,0) = 9999999;
    return angles;
  }
  if (Dsc > 0) {
    double root1 = 2*atan((-2*p1+sqrt(Dsc))/(2*(p3-p2)));
    double root2 = 2*atan((-2*p1-sqrt(Dsc))/(2*(p3-p2)));
    angles(0,0) = 2;
    angles(1,0) = root1 * (180 / PI);
    angles(2,0) = root2 * (180 / PI);
    return angles;
  }
  if (A*r(0,1) + B*r(1,1) + C*r(2,1) == 0) {
    angles(0,0) = 2;
    angles(1,0) = 90;
    angles(2,0) = 9999999;
    return angles;
  }
  double root = atan(-(A*r(0,0) + B*r(1,0) + C*r(2,0))/(A*r(0,1) + B*r(1,1) + C*r(2,1)));
  angles(0,0) = 2;
  double angle = root * (180 / PI);
  angles(1,0) = angle;
  if (angle < 0) {
    angles(2,0) = angle + 180;
  } else {
    angles(2,0) = angle - 180;
  }
  return angles;
}

bool isAngleInValidRange(double angle){
  double minAngle = 10;
  double maxAngle = 170;
  return angle >= minAngle && angle <= maxAngle;
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

double convertAngle(double angle) {
  return 180.0 - angle;
}

double angleBetweenVectors(BLA::Matrix<3,1> a, BLA::Matrix<3,1> b) {
  double a1 = a(0,0);
  double a2 = a(1,0);
  double a3 = a(2,0);
  double b1 = b(0,0);
  double b2 = b(1,0);
  double b3 = b(2,0);
  double len1 = sqrt(a1*a1 + a2*a2 + a3*a3);
  double len2 = sqrt(b1*b1 + b2*b2 + b3*b3);
  double dot = a1*b1 + a2*b2 + a3*b3;
  double cos = dot / (len1 * len2);
  return acos(cos) * (180.0 / 3.141592);
}

double sign(double value) {
  if (value < 0) {
    return -1;
  }
  return 1;
}

BLA::Matrix<3,1> angles = {0,0,0};
BLA::Matrix<3,3> rotationMatrix = {0,0,0,0,0,0,0,0,0};
BLA::Matrix<3,1> vector = {0,1,0};
BLA::Matrix<3,1> rotatedVector = {0,0,0}; 

bool calibrated = false;
BLA::Matrix<4,4> calMat1 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
BLA::Matrix<4,4> calMat2 = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void setup() {
 
  Serial.begin(9600);
  servo.setTargetAngle(convertAngle(90.0));

  mpu.setAddress(0x68);
  mpu2.setAddress(0x69);
  Wire.begin();
  
  byte status = mpu.begin(1,0,1);
  byte status2 = mpu2.begin(1,0,3);
 
  while(status!=0 && status2!=0){ } // stop everything if could not connect to MPU6050
  Serial.print(F("MPU6050 1 status: "));
  Serial.println(status);
  Serial.print(F("MPU6050 2 status: "));
  Serial.println(status2);
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  Serial.println("Done!\n");

  timer = millis();
}

int prevAngle = 0;
int currentAngle = 10;
int currentRawAngle = 10;
int firstFrame = true;
int currentHandAngle = 90;
bool setMotorPhysicalAngle = false;
int state = 0;


void loop() {
  if (setMotorPhysicalAngle) {
    servo.setTargetAngle(convertAngle(20.0));
    servo.update();
    double a = servo.getCurrentAngle();
    double aC = convertAngle(a);
    Serial.println(aC);
    return;
  }

  mpu.update();
  mpu2.update();

  //Hand
  BLA::Matrix<3,1> mpuHandAnglesRaw = {mpu2.getAngleX(), mpu2.getAngleY(), 0};
  BLA::Matrix<3,1> mpuHandAnglesProcessed = {0,90 + mpu2.getAngleY(),mpu2.getAngleX()};
  BLA::Matrix<4,4> mpuHandRot4 = eulerAnglesToMatrix(mpuHandAnglesProcessed, EEulerOrder::ORDER_YXZ);
  //Head
  BLA::Matrix<3,1> mpuHeadAnglesRaw = {mpu.getAngleX(), mpu.getAngleY(), 0};
  BLA::Matrix<3,1> mpuHeadAnglesProcessed = {mpu.getAngleX(), mpu.getAngleY(), 0};
  BLA::Matrix<4,4> mpuHeadRot4 = eulerAnglesToMatrix(mpuHeadAnglesProcessed, EEulerOrder::ORDER_YXZ);

  if (!calibrated && (millis()-timer)>3000) {
      calMat1 = Inverse(mpuHeadRot4);
      calMat2 = Inverse(mpuHandRot4);
      calibrated = true;
  }

  if (!calibrated) {
    return;
  }
 
  BLA::Matrix<4,4> mpuHandForward = {1,0,0,0,
                                     0,1,0,1,
                                     0,0,1,0,
                                     0,0,0,1};
  BLA::Matrix<4,4> mpuHandJointMat = mpuHandRot4 * calMat2 *  mpuHandForward;
  BLA::Matrix<3,1> mpuHandJointVec = getTranslation(mpuHandJointMat);

  BLA::Matrix<4,4> mpuHeadUp = {1,0,0,0,
                                0,1,0,0,
                                0,0,1,1,
                                0,0,0,1};
  BLA::Matrix<4,4> mpuHeadForward = {1,0,0,0,
                                    0,1,0,1,
                                    0,0,1,0,
                                    0,0,0,1};                              
  BLA::Matrix<4,4> mpuHeadJointMat = mpuHeadRot4 * calMat1 *  mpuHeadUp;
  BLA::Matrix<3,1> mpuHeadJointVec = getTranslation(mpuHeadJointMat);
  BLA::Matrix<4,4> mpuHeadForwardJointMat = mpuHeadRot4 * calMat1 *  mpuHeadForward;
  BLA::Matrix<3,1> mpuHeadForwardJointVec = getTranslation(mpuHeadForwardJointMat);

  double h = 0.22;
  double a1 = 0.30;
  double a2 = 0.39;
  double A = mpuHeadJointVec(0,0);
  double B = mpuHeadJointVec(1,0);
  double C = mpuHeadJointVec(2,0);
  double D = -C*h; 
  BLA::Matrix<4,1> N = {A,B,C,D};
  BLA::Matrix<3,1> angles1 = angleToSurface(N,mpuHandRot4,a1,a2);

  BLA::Matrix<3,1> downVector = {0,0,-1};
  double newCurrentHandAngle = angleBetweenVectors(downVector, mpuHandJointVec);
  BLA::Matrix<3,1> forwardVector = {0,1,0};
  double newCurrentHeadAngle = 90 + angleBetweenVectors(forwardVector, mpuHeadForwardJointVec) * sign(mpuHeadForwardJointVec(2,0));
  double aimZoneAngle = 25;

  double newAngle = 99999999;
  double angle1;
  double angle2;
  double avgAngle;

  switch (state) {
  case 0:   //START
    angle1 = angles1(1,0);
    angle2 = angles1(2,0);
    if (isAngleInValidRange(angle1) && isAngleInValidRange(angle2)) {
      //Both angles are valid
      newAngle = furthestOfTwoAngles(angle1, angle2);
    } else {
      if (isAngleInValidRange(angle1)) {
        newAngle = angle1;
      }
      if (isAngleInValidRange(angle2)) {
        newAngle = angle2;
      }
    }
    if (isAngleInValidRange(newAngle)) {
      currentAngle = newAngle;
    } else {
      currentAngle = 45;
    }
    state = 1;
    break;
  case 1:   //AIM MODE
    angle1 = angles1(1,0);
    angle2 = angles1(2,0);
    if (isAngleInValidRange(angle1) && isAngleInValidRange(angle2)) {
      //Both angles are valid
      newAngle = furthestOfTwoAngles(angle1, angle2);
    } else {
      if (isAngleInValidRange(angle1)) {
        newAngle = angle1;
      }
      if (isAngleInValidRange(angle2)) {
        newAngle = angle2;
      }
    }
    if (!isAngleInValidRange(newAngle) || abs(currentAngle - newAngle) > 45) {
      newAngle = currentAngle;
    }
    currentAngle = newAngle;
    if (newCurrentHandAngle < aimZoneAngle) {
      state = 2;
    }
    break;
  case 2:   //EXIT AIM
    currentAngle = 15;
    state = 3;
    break;
  case 3:   //FREE HAND
    currentAngle = 15;
    if (newCurrentHandAngle > aimZoneAngle) {
      state = 4;
    }
    break;
  case 4:   //ENTER AIM
    state = 0;
    break;      
  }
  servo.setTargetAngle(convertAngle(currentAngle));
  servo.update();
}
