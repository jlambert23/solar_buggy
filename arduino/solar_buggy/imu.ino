#include "RollingSum.h"

#define DECLINATION -6.58 // +- 0.3 In degrees, son
#define HEADING_OFFSET 187

RollingSum rs_imu(5);
float heading;

void read_imu() {   
  if (IMU_SERIAL.available() > 0) {
    heading = IMU_SERIAL.parseFloat() + DECLINATION + HEADING_OFFSET;
    if (heading > 360 || heading < 0) return;
    rs_imu.push(heading);
    Heading = rs_imu.sum / rs_imu.length - HEADING_OFFSET;
  }
}
