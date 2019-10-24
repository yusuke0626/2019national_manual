# GY521
GY-521をRasPiでI2C通信で使うときのライブラリ

## リファレンスのようなもの

```cpp
RPGY521::GY521()
```
* コンストラクタ
* そのうち可変長引数にする
* 静止状態で実行すること
* init()が成功するまで繰り返し実行する

```cpp
bool RPGY521::GY521::init()
```
* 静止状態で実行すること
* 角速度のレンジの設定, キャリブレーション(傾き・ドリフト効果)をしている

```cpp
void RPGY521::GY521::start()
```
* getYaw()をつかう前に呼び出してください

```cpp
double RPGY521::GY521::getYaw()
```
* start()を実行してからの累積のヨー軸の回転角度が返ってくる

```cpp
double RPGY521::GY521:diffYaw()
```
* 前回からのヨー軸の角度の差分が返ってくる

```cpp
void RPGY521::GY521::resetYaw(double reset)
```
* ヨー軸の累積値をresetの角度(degree)でリセットする
