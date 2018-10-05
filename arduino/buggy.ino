#include <ArduinoJson.h>
#include <NMEAGPS.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// IMU prep variables
LSM9DS1 imu;
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED
#define PRINT_SPEED 100 
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

// GPS prep variables.
NeoGPS::Location_t destination( 28.5853905, -81.2001379 );
static NMEAGPS  gps;
static gps_fix  fix;

// Ultrasonic config parameters and prep variables.
const int   ultrasonics = 4;
const int   echoPins[] = { 4, 6, 8, 10 };
const int   trigPins[] = { 5, 7, 9, 11 };
static long duration;
static int  distance;
static unsigned long lastPrint = 0; 

// Json output objects
DynamicJsonBuffer jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& gpsJson = root.createNestedObject("gps");
JsonObject& destJson = gpsJson.createNestedObject("waypoint");
JsonObject& ultraJson = root.createNestedObject("ultrasonic");

void setup() {
  for (int i = 0; i < ultrasonics; i++) {
    pinMode(echoPins[i], INPUT);
    pinMode(trigPins[i], OUTPUT);
  }
  
  Serial.begin(9600);
  Serial1.begin(38400); // GPS serial
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with IMU.");
    while (1)
      ;
  }
}

void loop() {  
  while (gps.available( Serial1 )) {
    fix = gps.read();

    gpsJson["latitude"] = fix.latitude();
    gpsJson["longitude"] = fix.longitude();

    destJson["distance"] = fix.location.DistanceMiles(destination);
    destJson["bearing"] = fix.location.BearingToDegrees(destination);

    // Syncing the ultrasonic to gps sentences.
    ultrasonic();
  
      // Update the sensor values whenever new data is available
    if ( imu.gyroAvailable() )
    {
      imu.readGyro();
    }
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
    if ( imu.magAvailable() )
    {
      imu.readMag();
    }
  
    if ((lastPrint + PRINT_SPEED) < millis())
    {
      printAttitude(imu.ax, imu.ay, imu.az, 
                   -imu.my, -imu.mx, imu.mz);
      // Serial.println();
      
      lastPrint = millis(); // Update lastPrint time
    }

      root.printTo(Serial);
      Serial.println();
    }
}

void ultrasonic()
{
  // Read and print all ultrasonic sensors.
  for (int i = 0; i < ultrasonics; i++) {
    digitalWrite(trigPins[i], LOW);
    delay(1);
    digitalWrite(trigPins[i], HIGH);
    delay(1);
    digitalWrite(trigPins[i], LOW);
    
    duration = pulseIn(echoPins[i], HIGH);
    distance = duration*0.034/2;

    ultraJson[String(i)] = distance;
  }
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  // Serial.print("Heading: "); Serial.println(heading, 2);

  gpsJson["IMU"] = heading;