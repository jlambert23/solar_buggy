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
// 28.5848326, -81.1999991
float latLong1[] = {28.5853905, -81.2001379};
float latLong2[] = {28.5848326, -81.1999991};
float latLong3[] = {28.60027778, -81.19805556}; // UCF behind HEC
float latLong4[] = {28.71, -81.41611111}; // Longwood

float Latitude = 0;
float Longitude = 0;
NeoGPS::Location_t destination( Latitude, Longitude );

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
      getLatLong(c);
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
