static NMEAGPS  gps;

byte baudrateCmd[] = {160, 161, 00, 04, 05, 00, 05, 00, 00, 13, 10}; // 115200    A0 A1 00 04 05 00 05 00 00 0D 0A
byte updateRate20HzCmd[] = {160, 161, 00, 03, 14, 20, 00, 26, 13, 10}; // 20 Hz     A0 A1 00 03 0E 14 00 1A 0D 0A
byte updateRate10HzCmd[] = {160, 161, 00, 03, 14, 10, 00, 04, 13, 10}; // 10 Hz     A0 A1 00 03 0E 0A 00 04 0D 0A

char buffer[40]; // For input mode
String output;
int i = 0;

void setup_GPS() {
  GPS_SERIAL.begin(38400);
  
  GPS_SERIAL.write(baudrateCmd, sizeof(baudrateCmd));
  GPS_SERIAL.flush();
  delay(10);
  GPS_SERIAL.begin(115200); // New baudrate
  
  GPS_SERIAL.write(updateRate10HzCmd, sizeof(updateRate10HzCmd));
}

void read_gps() {
  while (gps.available( Serial1 ))
    Fix = gps.read();
}

void getLatLong(char c) {
  if (c == ',') {
    buffer[i] = '\0';
    output += String(buffer) + ",";
    Latitude = atof(buffer);
    i = 0;
  }
  else if (c == '\n') {
    buffer[i] = '\0';
    output += String(buffer);
    Longitude = atof(buffer);
    i = 0;
    
    destination = NeoGPS::Location_t(Latitude, Longitude);
    LOG_PORT.println(output);
    output = "";
  }
  else
    buffer[i++] = c;
}
