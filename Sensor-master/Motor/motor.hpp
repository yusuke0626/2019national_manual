#include "../Gpio/gpio.hpp"

#ifndef MOTOR_MOTOR_HPP
#define MOTOR_MOTOR_HPP
template <class Port> class motor {
  get_hundle<Port> hundle;
  double multiplier;

public:
  motor(const get_hundle<Port> &hundle, double multiplier = 1.0)
      : hundle(hundle), multiplier(multiplier) {}

  template <class Num> auto roll(const Num value) const {
    return hundle.send(value * multiplier);
  }

  void set_multiplier(const double multiplier_) { multiplier = multiplier_; }
};
#endif
