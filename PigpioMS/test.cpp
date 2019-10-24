// サンプルコード兼動作テスト用のコードのつもり
// 使い方 : makeでビルド出来ます. sudo ./testで実行
// 引数に送信したいid cmd dataを10進数で入れるとそれを送ります.
// 引数が足りないとid=1, cmd=2, data=255で通信を行います.
// 最初に送信するデータを表示し, 通信が終わると結果を表示します.

#include "PigpioMS.hpp"
#include <iostream>

using namespace std;
using namespace RPMS;

int main(int argc, char *argv[]) {
  MotorSerial ms; // すべてデフォルト値でインスタンスを生成
  ms.setTimeOut(1000); // 1秒のタイムアウトを設定
  try {
    ms.init(); // 初期化
  } catch (runtime_error exception) {
    cout << "Setup Error" << endl;
    return -1;
  }
  unsigned char id, cmd;
  unsigned short data;
  if (argc < 4) {
    id = 1;
    cmd = 2;
    data = 255;
  } else {
    id = (unsigned char)stoi(argv[1]);
    cmd = (unsigned char)stoi(argv[2]);
    data = (short)stoi(argv[3]);
  }
  cout << (int)id << " " << (int)cmd << " " << (int)data << endl;
  try {
    cout << ms.send(id, cmd, data) << endl; // 同期モードで通信
  } catch (runtime_error exception) {
    cout << "Communication Error." << endl;
    return -1;
  }
  cout << (ms.sumCheckSuccess ? "Receive Success" : "Receive Failed")
       << endl; // 通信の成否を確認, 表示
  return 0;
}
