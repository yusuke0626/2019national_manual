#include <iostream>
#include <map>

#ifndef GPIO_GPIO_HPP
#define GPIO_GPIO_HPP
/* `cout << a << b << ...` めんどくさいので補助関数 */
template <class... Ts> void println(Ts... xs) {
  (std::cout << ... << xs);
  std::cout << std::endl;
}

/*出力の下請け*/
template <class Port> class get_hundle {
  Port &port;
  unsigned char id;
  unsigned char cmd;

public:
  get_hundle(Port &port, const unsigned char id, const unsigned char cmd)
      : port(port), id(id), cmd(cmd) {}

  auto send(const short value) const { return port.send(id, cmd, value); }
};

/*id, cmdをget_hundleに渡すset関数のテンプレ*/
template <class T> class port_base {
public:
  get_hundle<T> set(const unsigned char id, const unsigned char cmd) {
    return {*static_cast<T *>(this), id, cmd};
  }
};

inline namespace gpio {
class serial_base {};

class async_serial : public port_base<async_serial> {
  const char *path;

public:
  async_serial(const char *path = "/dev/serial1") : path(path) {}

  bool send(const unsigned char id, const unsigned char cmd,
            const short value) {
    println("[ASYNC_SERIAL]", "path: \"", path, "\"; id: ", +id, "; cmd: ",
            +cmd, "; value: ", value);
    return true;
  }
};

class sync_serial : public port_base<sync_serial> {
  const char *path;

public:
  sync_serial(const char *path = "/dev/serial1") : path(path) {}

  int send(const unsigned char id, const unsigned char cmd, const short value) {
    println("[SYNC_SERIAL]", "path: \"", path, "\"; id: ", +id, "; cmd: ", +cmd,
            "; value: ", value);
    return value;
  }
};
}
#endif
