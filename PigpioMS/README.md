# RasPiMoterSerial

**現在[RobotUtil4RasPi](https://github.com/Owl8/RobotUtilityforRaspberryPi)のScrpMasterに移植しています.そちらを使うようにしてください**
* 先輩の作ったモータードライバとかのネットワークで便利に通信したいとおもって作った
* [SCRP](https://github.com/okayu9/SCRP)に従ってRaspberryPi等をマスタとして通信するためのライブラリみたいなもの
* 通信相手は[これ](https://github.com/okayu9/ScrpMotor)を書き込んだArduinoのUARTをRS485に変換したもののネットワークを想定しています
* WiringPiを使ってます
* threadとか使ってます C++11以降のコンパイラでビルドしてください
* RPMS::MotorSerialクラスのインスタンスを作って使ってください
* インスタンス一つにつきシリアルポート一つの気持ちで使ってね

* そのうちもっとわかり易い名前にしてスレーブとして使う機能も付けて作りなおす予定

* 2018.6.20追記:wiringPiからpigpioに移行しました

## リファレンス的な何か
### MotorSerialクラス
```cpp
RPMS::MotorSerial(int ReDe = 4, double timeout = 0.01, consta char * devFileName = "/dev/ttyAMA0")
```
* コンストラクタです.
* 引数は, REピンとDEピンの指定, タイムアウトの指定(ms), デバイスのファイル名

```cpp
void RPMS::MotorSerial::init()
```
* GPIOの初期化とか、シリアル通信のOpenなどを行います。インスタンスにつき一度だけ最初に呼んでください.

```cpp
short RPMS::MotorSerial::sending(unsigned char id, unsigned char cmd, short data)
```
* id番の基板に対してcmd, dataを送信します.返り値としてid番の基板から返ってきたdataを返します.

```cpp
short RPMS::MotorSerial::send(unsigned char id, unsigned char cmd, short data, bool asyncFlag = false)
```
* asyncFlag = falseならsending()と同じ動作をします.asyncFlag = trueなら非同期で通信してくれます。その場合返り値は0です.
* sendingよりもこっちを使っとけばタイプ数も少なくて済むし非同期・同期の切り替えも楽だよ.
* RaspberryPiはシングルコアなので, 非同期モードで短時間に大量のデータを送るようなコードを書くと通信を行うスレッドの処理が行われず,想定してるのと異なる動作をする場合があります.注意してください.

```cpp
bool RPMS::MotorSerial::sumCheckSuccess
```
* 直近の通信が受信のsumチェックの成功まで行ったかどうかの結果が入っています.

```cpp
short RPMS::MotorSerial::recentReceiveData
```
* 直近の通信で受信したdataが入っています.
* send()を非同期モードで使ったが結果がほしい時とかに使うといいのかもしれない.

```cpp
static bool RPMS::MotorSerial::wiringPiSetupGpioFlag
```
* wiringPiSetupGpio()を呼んだかどうかが入っています.
* init()より前にwiringPiSetupGpio()を呼んでいる場合とかにtrueを代入しておくと良いかもしれない.

### Motorクラス
おまけ的存在。MotorSerialを用いてモーターを制御したい時に便利かもしれない

```cpp
typedef struct {
  unsigned char id;
  unsigned char mNum;
  double magni;
} RPMS::MotorDataFormat;
```
* モーターの設定の構造体
* 主にloadMotorSetting()で読み込んだ設定を保存しMotorクラスのコンストラクタに渡すためにある

```cpp
int RPMS::loadMotorSetting(char* settingFileName, MotorDataFormat* MotorDatas, int MotorsNum);
```
* モーターの設定を読み込む関数
* settingFileNameには相対パスでファイル名を渡す。
* MotorDatasに読み込んだ設定を保存するためのMotorDataFormatの配列、MotorsNumにモーターの数を入れる。
* 設定ファイルの書き方は、1行につきモーター1つで"ID モーター番号 倍率"みたいな感じでスペース区切りで設定を書く。数字または-以外で始まっているデータは無視されます。

```cpp
RPMS::Motor::Motor(unsigned char id, unsigned char mNum, double magni, MotorSerial* ms, short maxPower = MaxMotorPower);
```
* Motorクラスのコンストラクタ
* idにモータードライバドライバのID, mNumにモーター番号, magniにモーターを回す時の倍率を入れる。
* magniはモーターを回す直前に掛けられる数なのでこれで回転方向とか制御できる
* msにMotorSerialクラスのポインタ、maxPowerにモーターの回転数の最大値を入れる
* MaxMotorPowerはいじれるようにしてある

```cpp
RPMS::Motor::Motor(MotorDataFormat, MotorSerial* short maxPower = MaxMotorPower);
```
* 引数に構造体をとるタイプのコンストラクタ
* loadMotorSetting()とか使うときにはこっち

```cpp
short RPMS::Motor::changeMaxPower(short maxPower);
```
* モーターの出力の最大値を変更するメソッド

```cpp
virtual bool RPMS::Motor::spin(short power);
```
* モーターを回すメソッド
* powerにpwm値を入れる。負値で反転
* 絶対値がmaxPower以上の値が入るとmaxPowerにされて出力される
* 通信が成功したか失敗したかがboolで返ってくる

```cpp
virtual bool RPMS::Motor::~Motor();
```
* デストラクタ
* id mNum 0 みたいな感じのデータをモータードライバドライバに送信してモーターを止める。
