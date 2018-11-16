#include "RollingSum.h"

#define DECLINATION -6.58 // +- 0.3 In degrees, son

RollingSum rs_imu(10);
String buffer;

void read_imu() {   
  if (IMU_SERIAL.available() > 0) {
    rs_imu.push(IMU_SERIAL.parseFloat() + DECLINATION);
    Heading = rs_imu.sum;
  }
}
