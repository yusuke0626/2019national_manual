#include "PigpioMS.hpp"
#include <chrono>
#include <cstring>
#include <mutex>
#include <pigpio.h>
#include <pthread.h>
#include <queue>
#include <string>
#include <thread>
#include <unistd.h>

#include <iostream>

constexpr int STX = 0x41;

using namespace std;
using namespace RPMS;

constexpr int SEND_DATA_NUM = 7;
int RPMS::MaxMotorPower = 200;

bool MotorSerial::pigpioSetupGpioFlag = false;

MotorSerial::MotorSerial(int rede, int timeout, const char *devFileName,
                         int bRate) {
  init(rede, timeout, devFileName, bRate);
}

MotorSerial::MotorSerial() { init(4, 10, "/dev/ttyAMA0", 115200); }

void MotorSerial::init(int rede, int timeout, const char *devFileName,
                       int bRate) {
  sumCheckSuccess = false;
  recentReceiveData = 0;
  threadLoopFlag = false;
  this->serialFileName = (char *)devFileName;
  this->bRate = bRate;
  this->timeOut = timeout;
  this->redePin = rede;
}

void MotorSerial::init() {
  unsigned char serFlags = 0;
  if (!pigpioSetupGpioFlag) {
    if (gpioInitialise() < 0) {//2018/7/5 (gpioInitialise() < 0)
      serClose(serialFile);
      throw runtime_error("WiringPiSetupError");
    }
    pigpioSetupGpioFlag = true;
  }
  this->serialFile = serOpen(serialFileName, bRate, serFlags);
  if (serialFile < 0) {
    throw runtime_error("SerialOpenError");
  }
  gpioSetMode(this->redePin, PI_OUTPUT);
}

void MotorSerial::setTimeOut(int timeout) { timeOut = timeout; }

short MotorSerial::sending(unsigned char id, unsigned char cmd, short data) {
  unsigned short uData = (unsigned short)data;
  unsigned char sendArray[SEND_DATA_NUM] = {
      0xFF,
      STX,
      id,
      cmd,
      (unsigned char)(uData & 0xFF),
      (unsigned char)(uData >> 8),
      (unsigned char)((id + cmd + (uData & 0xFF) + (uData >> 8)) & 0xFF)};
  lock_guard<mutex> lock(mtx);

  gpioWrite(this->redePin, 1);
  for (int i = 0; i < SEND_DATA_NUM; ++i) {
    serWriteByte(serialFile, sendArray[i]);
    gpioDelay(90);
  }
  gpioWrite(this->redePin, 0);

  bool stxFlag = false;
  char receiveArray[5] = {};
  int i = 0;

  auto startTime = chrono::system_clock::now();
  sumCheckSuccess = false;
  while (chrono::time_point<chrono::system_clock>(
             startTime + chrono::milliseconds(timeOut)) >=
             chrono::system_clock::now() &&
         !sumCheckSuccess) {
    while (serDataAvailable(serialFile) > 0) {
      char gotData = serReadByte(serialFile);
      if (gotData == STX && !stxFlag) {
        stxFlag = true;
        continue;
      }
      if (stxFlag) {
        receiveArray[i++] = gotData;
      }
      if (i > 4) {
        unsigned char sum = 0;
        for (int j = 0; j < 4; ++j)
          sum += receiveArray[j];
        if (sum == receiveArray[4]) {
          sumCheckSuccess = true;
          break;
        }
      }
    }
  }
  if (serDataAvailable(serialFile) < 0) {
    throw runtime_error("SerialComError");
  }
  return (recentReceiveData = receiveArray[2] + (receiveArray[3] << 8));
}

void MotorSerial::sendingLoop(void) {
  while (!sendDataQueue.empty()) {
    threadLoopFlag = true;
    SendDataFormat sendData = sendDataQueue.front();
    sendDataQueue.pop();
    sending(sendData.id, sendData.cmd, sendData.argData);
  }
  threadLoopFlag = false;
}

short MotorSerial::send(unsigned char id, unsigned char cmd, short data,
                        bool asyncFlag) {
  if (asyncFlag) {
    SendDataFormat sendData = {id, cmd, data};
    sendDataQueue.push(sendData);
    if (!threadLoopFlag) {
      if (sendThread.joinable())
        sendThread.join();
      sendThread = thread([&] { sendingLoop(); });
      sched_param sch_params;
      sch_params.sched_priority = 1;
      if (pthread_setschedparam(sendThread.native_handle(), SCHED_RR,
                                &sch_params)) {
        cerr << "Failed to set Thread scheduling :" << strerror(errno) << endl;
      }
    }
    return 0;
  }
  return sending(id, cmd, data);
}

short MotorSerial::send(SendDataFormat sendData, bool asyncFlag) {
  return send(sendData.id, sendData.cmd, sendData.argData, asyncFlag);
}

MotorSerial::~MotorSerial() {
  if (sendThread.joinable())
    sendThread.join();
  serClose(this->serialFile);
  gpioTerminate();
}

Motor::Motor() { initFlag = false; }

Motor::Motor(unsigned char id, unsigned char mNum, double magni,
             MotorSerial *ms, short maxPower) {
  this->id = id;
  this->mNum = mNum;
  this->magni = magni;
  this->ms = ms;
  this->maxPower = maxPower;
  initFlag = true;
}

Motor::Motor(MotorDataFormat MotorData, MotorSerial *ms, short maxPower) {
  this->id = MotorData.id;
  this->mNum = MotorData.mNum;
  this->magni = MotorData.magni;
  this->ms = ms;
  this->maxPower = maxPower;
  initFlag = true;
}

short Motor::changeMaxPower(short maxPower) {
  return (this->maxPower = maxPower);
}

bool Motor::spin(short motorPower, bool asyncFlag) {
  if (!initFlag)
    return false;
  motorPower = (short)(magni * motorPower);
  if (motorPower > maxPower)
    motorPower = maxPower;
  if (motorPower < -maxPower)
    motorPower = -maxPower;
  return motorPower == ms->send(id, 2 + mNum, motorPower, asyncFlag);
}

Motor::~Motor() { spin(0); }
int RPMS::loadMotorSetting(char *FileName, RPMS::MotorDataFormat *MotorDatas,
                           int NumMotors) {
  // 現在のパスを取得
  char buf[512] = {};
  readlink("/proc/self/exe", buf, sizeof(buf) - 1); // 実行ファイルのパスを取得
  string path(buf);
  path += FileName;
  // モーターの設定を読み込む
  ifstream settingFile;
  settingFile.open(path);
  if (!settingFile)
    return -1;
  for (int i = 0; (i < NumMotors) && (!settingFile.eof()); ++i) {
    for (int j = 0; j < 3 && (!settingFile.eof()); ++j) {
      string str;
      char *ends;
      settingFile >> str;
      if ((str.c_str()[0] >= '0' && str.c_str()[0] <= '9') ||
          (j == 2 && str.c_str()[0] == '-')) {
        switch (j) {
        case 0:
          MotorDatas[i].id = atoi(str.c_str());
          break;
        case 1:
          MotorDatas[i].mNum = atoi(str.c_str());
          break;
        case 2:
          MotorDatas[i].magni = strtod(str.c_str(), &ends);
          break;
        }
      }
    }
  }
  return 0;
}
