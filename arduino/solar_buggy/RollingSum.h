#ifndef RollingSum_h
#define RollingSum_h

#include "Arduino.h"

class RollingSum {
  private:
    int _size;
    int _index;
    float   *_queue;

  public:
    int length;
    float sum;

    RollingSum(int size);
    ~RollingSum();
    void push(float value);
    void print(HardwareSerial s);
};

#endif
