#include <ArduinoJson.h>
#include <NMEAGPS.h>

NeoGPS::Location_t destination( 28.5853905, -81.2001379 );

// GPS prep variables.
static NMEAGPS  gps;
static gps_fix  fix;

// Ultrasonic config parameters and prep variables.
const int   ultrasonics = 4;
const int   echoPins[] = { 4, 6, 8, 10 };
const int   trigPins[] = { 5, 7, 9, 11 };
static long duration;
static int  distance;

// Json output objects
DynamicJsonBuffer jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& gpsJson = root.createNestedObject("gps");
JsonObject& destJson = gpsJson.createNestedObject("waypoint");
JsonObject& ultraJson = root.createNestedObject("ultrasonic");

byte baudrateCmd[] = {160, 161, 00, 04, 05, 00, 05, 00, 00, 13, 10}; // 115200		A0 A1 00 04 05 00 05 00 00 0D 0A
byte updateRate20HzCmd[] = {160, 161, 00, 03, 14, 20, 00, 26, 13, 10}; // 20 Hz			A0 A1 00 03 0E 14 00 1A 0D 0A
byte updateRate10HzCmd[] = {160, 161, 00, 03, 14, 10, 00, 04, 13, 10}; // 10 Hz			A0 A1 00 03 0E 0A 00 04 0D 0A


void setup() {
  for (int i = 0; i < ultrasonics; i++) {
    pinMode(echoPins[i], INPUT);
    pinMode(trigPins[i], OUTPUT);
  }
  
  Serial.begin(9600);
  Serial1.begin(38400); // GPS serial
  
  Serial1.write(baudrateCmd, sizeof(baudrateCmd));
  Serial1.flush();
  delay(10);
  Serial1.begin(115200); // New baudrate
  
  Serial1.write(updateRate10HzCmd, sizeof(updateRate10HzCmd));
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

    root.printTo(Serial);
    Serial.println();
  }
  
}

void ultrasonic()
{
  // Read and print all ultrasonic sensors.
  for (int i = 0; i < ultrasonics; i++) {
    
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);
    
    duration = pulseIn(echoPins[i], HIGH);
    
    distance = duration*0.034/2;
    distance = distance > 1000 ? 350 : distance;

    ultraJson[String(i)] = distance;
  }
}
