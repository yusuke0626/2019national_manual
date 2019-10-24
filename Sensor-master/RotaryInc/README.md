# rotaryInc
RasPiでインクリメント型ロータリーエンコーダ(2相出力)を使う

## リファレンスのようなもの
```cpp
rotaryInc::rotaryInc(int userA, int pinB, bool precision)
```
* コンストラクタ
* userA, userB でA相, B相のGPIOピンの指定
* recision == true なら2逓倍になる
* そのうち4逓倍もつける

```cpp
int rotaryInc::get()
```
* 返り値はパルスが来た回数
* これを分解能で割ってやると回転数になる
