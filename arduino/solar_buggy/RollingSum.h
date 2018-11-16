#ifndef RollingSum_h
#define RollingSum_h

#include "Arduino.h"

class RollingSum {
  private:
    int _size;
    int _index;
    int *_queue;

  public:
    int length;
    float sum;

    RollingSum(int size);
    ~RollingSum();
    void push(float value);
};

#endif