#pragma once
#include <stdint.h>

class rotaryInc {
public:
  rotaryInc(int userA, int userB, bool precision);
  int get();

private:
  int pulse = 0;
  int pinA, pinB;
  bool nowA, nowB;
  static void rotary(int gpio, int level, uint32_t tick, void *userdata);
  static void rotaryEx(int gpio, int level, uint32_t tick, void *userdata);
};
