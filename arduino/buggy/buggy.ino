#include <ArduinoJson.h>
#include <NMEAGPS.h>

NeoGPS::Location_t destination( 28.58502, -81.19885 );

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

void setup() {
  for (int i = 0; i < ultrasonics; i++) {
    pinMode(echoPins[i], INPUT);
    pinMode(trigPins[i], OUTPUT);
  }
  
  Serial.begin(9600);
  Serial1.begin(38400); // GPS serial
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
    delay(1);
    digitalWrite(trigPins[i], HIGH);
    delay(1);
    digitalWrite(trigPins[i], LOW);
    
    duration = pulseIn(echoPins[i], HIGH);
    distance = duration*0.034/2;

    ultraJson[String(i)] = distance;
  }
}
