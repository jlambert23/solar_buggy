#include "Arduino.h"
#include "RollingSum.h"

RollingSum::RollingSum(int size) {
  _queue = (int*) calloc(sizeof(int) , size);
  _size = size;
  _index = 0;
  length = 0;
  sum = 0;
}

RollingSum::~RollingSum() { free(_queue); _queue = NULL; }

void RollingSum::push(float value) {
  sum += value - _queue[_index];
  _queue[_index] = value;
  ++_index %= _size;
  if (length < _size) length++;
};
