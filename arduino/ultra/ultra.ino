#include <ArduinoJson.h>

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
}

void loop() {  
  
  ultrasonic();
  root.printTo(Serial);
  Serial.println();
  
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
    
    ultraJson[String(i)] = distance/2.54;
  }
}
