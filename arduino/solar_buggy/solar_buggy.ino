#include <NMEAGPS.h>
#include <ArduinoJson.h>

#define LOG_PORT Serial
#define GPS_SERIAL Serial1
#define IMU_SERIAL Serial2

#define LOG_BAUDRATE 9600
#define IMU_BAUDRATE 57600

#define ULTRASONICS 4
const int echoPins[] = { 4, 6, 8, 10 };
const int trigPins[] = { 5, 7, 9, 11 };

// 28.5853905,-81.2001379
// 28.5845915,-81.1997349
float Longitude = 28.5853905;
float Latitude = -81.2001379;
NeoGPS::Location_t destination( Longitude, Latitude );

bool input_mode = false;

// Primary data variables.
static gps_fix Fix;
static float Heading;
static float Distance[4];

void setup() {  
  LOG_PORT.begin(LOG_BAUDRATE);
  IMU_SERIAL.begin(IMU_BAUDRATE);
  
  setup_GPS();
  setup_ultrasonic();
}

void loop() {
  if(LOG_PORT.available()) {
    char c = LOG_PORT.read();
    if (c == '#')
      handle_command();

    else if (input_mode)
      getLongLat(c);
  }

  else if (!input_mode) {
    read_gps();
    read_imu();
    read_ultrasonic();
    outputCSV();
  }
}

void handle_command() {
  while (LOG_PORT.available() < 1);
  char command = LOG_PORT.read();
  
  if (command == 'w')
    input_mode = true;
  else
    input_mode = false;
}
