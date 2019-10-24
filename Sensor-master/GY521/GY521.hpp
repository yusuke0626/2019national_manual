#pragma once
#include <pigpio.h>
#include <time.h>

namespace RPGY521 {
enum RegisterMap {
  WHO_AM_I = 0x75,
  PWR_MGMT_1 = 0x6B,
  FS_SEL = 0x1B,
  GYRO_ZOUT_H = 0x47,
  GYRO_ZOUT_L = 0x48,
  AFS_SEL = 0x1C,
  ACCEL_XOUT_H = 0x3B,
  ACCEL_XOUT_L = 0x3C,
  ACCEL_YOUT_H = 0x3D,
  ACCEL_YOUT_L = 0x3E,
  ACCEL_ZOUT_H = 0x3F,
  ACCEL_ZOUT_L = 0x40,
};
constexpr double LSBMap[4] = {131, 65.5, 32.8, 16.4};
class GY521 {
public:
  GY521();
  GY521(int dev, int bit, int calibration, double userReg);
  double yaw;
  double diffYaw;
  void updata();
  void resetYaw(double reset) { yaw = reset; }
  void start() {
    clock_gettime(CLOCK_REALTIME, &now);
    resetYaw(0);
  }
  void start(double start) {
    clock_gettime(CLOCK_REALTIME, &now);
    resetYaw(start);
  }
  ~GY521();

private:
  int devId;
  int I2cId;
  unsigned int i2cFlag;
  double gyroZAver;
  double gyroLSB;
  struct timespec now, prev;

  bool init(int dev, int bit, int calibration, double userReg);

  //マクロ的なやつ
  int gyroRead(enum RegisterMap Register) {
    return i2cReadByteData(I2cId, Register);
  }

  int gyroRead2(enum RegisterMap RegisterH, enum RegisterMap RegisterL) {
    return (gyroRead(RegisterH) << 8) + gyroRead(RegisterL);
  }

  bool gyroWrite(enum RegisterMap Register, int data) {
    return i2cWriteByteData(I2cId, Register, data) == devId ? 1 : 0;
  }
};
};
