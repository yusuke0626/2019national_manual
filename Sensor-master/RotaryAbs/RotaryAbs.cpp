#include "RotaryAbs.hpp"
#include <atomic>
#include <thread>
#include <wiringPi.h>

using namespace std;

E6CP::E6CP(int pin[8], atomic<bool> *flag) {
  // Decide PinNumber
  for (int i = 0; i < 8; i++) {
    absolute[i] = pin[i];
  }

  // Create Macro
  for (unsigned char i = 0;; ++i) {
    Change[i ^ (i >> 1)] = i;
    if (i == 255) {
      break;
    }
  }

  // Pin Config
  wiringPiSetupGpio();
  for (int i = 0; i < 8; ++i) {
    pinMode(absolute[i], INPUT);
  }

  wait = flag;
  wait->store(flag);
  loopFlag = true;
  readSpecialThread = thread(&E6CP::readSpecialLoop, this);
}

long E6CP::get() {
  static unsigned char rotaryDummy;
  rotaryDummy = 0;
  for (unsigned char i = 0; i < 8; ++i) {
    rotaryDummy += !digitalRead(absolute[i]) << i;
  }
  rotaryNow = Change[rotaryDummy];
  totalNow += rotaryNow - rotaryPrev;
  rotaryPrev = rotaryNow;
  return totalNow;
}

long E6CP::diff() {
  static unsigned char rotaryDummy;
  rotaryDummy = 0;
  for (unsigned char i = 0; i < 8; ++i) {
    rotaryDummy += !digitalRead(absolute[i]) << i;
  }
  rotaryPrev = rotaryNow;
  rotaryNow = Change[rotaryDummy];
  return rotaryNow - rotaryPrev;
}

void E6CP::offset(long value) { totalNow.store(value); }

void E6CP::readSpecial() {
  static unsigned char rotaryDummy;
  // 0 -> 255
  if (!digitalRead(absolute[7])) {
    rotaryDummy = 0;
    for (unsigned char i = 0; i < 8; ++i) {
      rotaryDummy += !digitalRead(absolute[i]) << i;
    }
    rotaryNow = Change[rotaryDummy];
    totalNow += rotaryNow - rotaryPrev;
    rotaryPrev = rotaryNow;
    if (rotaryPrev == 255) {
      totalNow -= 256;
    }
  }
  // 255 -> 0
  else {
    rotaryDummy = 0;
    for (unsigned char i = 0; i < 8; ++i) {
      rotaryDummy += !digitalRead(absolute[i]) << i;
    }
    rotaryNow = Change[rotaryDummy];
    totalNow += rotaryNow - rotaryPrev;
    rotaryPrev = rotaryNow;
    if (rotaryPrev == 0) {
      totalNow += 256;
    }
  }
}

void E6CP::readSpecialLoop() {
  while (loopFlag) {
    if (wait->load()) {
      readSpecial();
      wait->store(false);
    }
  }
}

E6CP::~E6CP() {
  loopFlag = false;
  readSpecialThread.join();
}
