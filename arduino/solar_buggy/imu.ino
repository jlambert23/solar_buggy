#define DECLINATION -6.58 // +- 0.3 In degrees, son

String buffer;
float heading_new;

void read_imu() {  
  while (IMU_SERIAL.available() > 0) {
    char c = IMU_SERIAL.read();
    buffer += c;

    if (c == '\n') {
      heading_new = buffer.toFloat() + DECLINATION;
      if (heading_new < 180) Heading = heading_new;
      buffer = "";
      return;
    }
  }
}
