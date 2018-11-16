#include "RollingSum.h"

// Parameters
#define MAX_DISTANCE 100 // in cm (For timeout; ~6500 mu)
#define TIMEOUT_BUFFER 600 // in mu
#define RS_ULTRA_SIZE 10

// Constants
#define SPEED_OF_SOUND 34.3 // in cm/ms
#define TIMEOUT MAX_DISTANCE / SPEED_OF_SOUND * 2000 + TIMEOUT_BUFFER // in mu

static long duration;
static float distance;
RollingSum rs_ultra[4] = {RollingSum(RS_ULTRA_SIZE),RollingSum(RS_ULTRA_SIZE),RollingSum(RS_ULTRA_SIZE),RollingSum(RS_ULTRA_SIZE)};

void setup_ultrasonic() {
  for (int i = 0; i < ULTRASONICS; i++) {
    pinMode(echoPins[i], INPUT);
    pinMode(trigPins[i], OUTPUT);
  }
}

void read_ultrasonic()
{
  // Read and print all ultrasonic sensors.
  for (int i = 0; i < ULTRASONICS; i++) {
    
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);
    
    duration = pulseIn(echoPins[i], HIGH, TIMEOUT);
    distance = duration * SPEED_OF_SOUND / 2000;
    rs_ultra[i].push(distance);
    Distance[i] = rs_ultra[i].sum / rs_ultra[i].length;
  }
}
