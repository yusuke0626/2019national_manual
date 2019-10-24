#pragma once

#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace RPMS {
extern int MaxMotorPower;

typedef struct {
  unsigned char id;
  unsigned char cmd;
  short argData;
} SendDataFormat;
typedef struct {
  unsigned char id;
  unsigned char mNum;
  double magni;
} MotorDataFormat;

class MotorSerial {
public:
  MotorSerial(int, int timeout = 10, const char *devFileName = "/dev/ttyAMA0",
              int bRate = 115200);
  MotorSerial();
  void init();
  void setTimeOut(int);
  short send(unsigned char, unsigned char, short, bool asyncFlag = false);
  short send(SendDataFormat, bool asyncFlag = false);
  short sending(unsigned char, unsigned char, short);
  virtual ~MotorSerial();
  bool sumCheckSuccess;
  short recentReceiveData;
  static bool pigpioSetupGpioFlag;
  //		bool serialReceiveSuccess;
private:
  void init(int, int, const char *, int);
  void sendingLoop(void);
  char *serialFileName;
  bool nowSendingFlag;
  bool threadLoopFlag;
  int timeOut;
  int serialFile;
  int bRate;
  int redePin;
  std::thread sendThread;
  std::queue<SendDataFormat> sendDataQueue;
  std::mutex mtx;
};
class Motor {
public:
  Motor();
  Motor(unsigned char, unsigned char, double, MotorSerial *,
        short maxPower = MaxMotorPower);
  Motor(MotorDataFormat, MotorSerial *, short maxPower = MaxMotorPower);
  short changeMaxPower(short);
  virtual bool spin(short, bool asyncFlag = false);
  virtual ~Motor();

protected:
  MotorSerial *ms;
  unsigned char id;
  unsigned char mNum;
  double magni;
  short maxPower;
  bool initFlag;
};
int loadMotorSetting(char *, MotorDataFormat *, int);
}
