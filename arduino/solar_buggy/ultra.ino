// Results in TIMEOUT ~ 6500 mu
#define MAX_DISTANCE 100 // in cm
#define TIMEOUT_BUFFER 600 // in mu

#define SPEED_OF_SOUND 34.3 // in cm/ms
#define TIMEOUT MAX_DISTANCE / SPEED_OF_SOUND * 2000 + TIMEOUT_BUFFER // in mu
#define ULTRA_THRESHOLD 20

static float old_ultrasonics[ULTRASONICS];
static float distance;
static long duration;

void setup_ultrasonic() {
  for (int i = 0; i < ULTRASONICS; i++) {
    pinMode(echoPins[i], INPUT);
    pinMode(trigPins[i], OUTPUT);
    old_ultrasonics[i] = 0;
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
 
    if (abs(old_ultrasonics[i] - distance) <= ULTRA_THRESHOLD)
      Distance[i] = distance <= 0 ? -1 : distance;
    
    old_ultrasonics[i] = distance;
  }
}
