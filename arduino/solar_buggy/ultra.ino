// Results in TIMEOUT ~ 6500 mu
#define MAX_DISTANCE 100 // in cm
#define TIMEOUT_BUFFER 600 // in mu

#define SPEED_OF_SOUND 34.3 // in cm/ms
#define TIMEOUT MAX_DISTANCE / SPEED_OF_SOUND * 2000 + TIMEOUT_BUFFER // in mu

static long duration;

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
    
    Distance[i] = duration*0.034/2;
    Distance[i] = Distance[i] <= 0 ? -1 : Distance[i];
  }
}
