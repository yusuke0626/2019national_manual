#include<thread>
#include<atomic>

class E6CP{
  public:
    E6CP(int pin[8], std::atomic<bool> *flag);
    long get();
    long diff();
    void offset(long value);
    ~E6CP();
  private:
    unsigned char Change[256] = {};
    int absolute[8] = {};
    unsigned char rotaryNow = 0;
    unsigned char rotaryPrev = 0;
    std::atomic<long> totalNow;
    std::thread readSpecialThread;
    void readSpecial();
    std::atomic<bool> *wait;
    void readSpecialLoop();
    bool loopFlag;
};
