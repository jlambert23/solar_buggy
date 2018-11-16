#include <NMEAGPS.h>
#include <ArduinoJson.h>

#define OUTPUT_SERIAL Serial
#define GPS_SERIAL Serial1
#define IMU_SERIAL Serial2

#define OUTPUT_BAUDRATE 9600
#define IMU_BAUDRATE 57600

#define ULTRASONICS 4
const int echoPins[] = { 4, 6, 8, 10 };
const int trigPins[] = { 5, 7, 9, 11 };

float LongLat[] = { 28.5853905, -81.2001379 };
float LongLat2[] = { 28.5845915, -81.1997349 };
NeoGPS::Location_t destination( LongLat[0], LongLat[1] );

// Primary data variables.
static gps_fix Fix;
static float Heading;
static float Distance[4];

void setup() {  
  OUTPUT_SERIAL.begin(OUTPUT_BAUDRATE);
  IMU_SERIAL.begin(IMU_BAUDRATE);
  
  setup_GPS();
  setup_ultrasonic();
}

void loop() {
  read_gps();
  read_imu();
  read_ultrasonic();
  outputCSV();
}
