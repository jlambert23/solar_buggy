#define DECLINATION -6.58 // +- 0.3 In degrees, son
#define IMU_THRESHOLD 10

String buffer;
float heading_new;
float heading_old = 0;

void read_imu() {   
  if (IMU_SERIAL.available() > 0) {
    heading_new = IMU_SERIAL.parseFloat() + DECLINATION;
    if (abs(heading_new - heading_old <= IMU_THRESHOLD)) Heading = heading_new;
    heading_old = heading_new;
  }
}
