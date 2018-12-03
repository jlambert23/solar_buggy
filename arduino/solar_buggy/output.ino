// Json output objects
DynamicJsonBuffer jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& gpsJson = root.createNestedObject("gps");
JsonObject& destJson = gpsJson.createNestedObject("waypoint");
JsonObject& ultraJson = root.createNestedObject("ultrasonic");

void outputCSV() {
  // ultrasonic_0, ultrasonic_1, ultrasonic_2, ultrasonic_3, latitude, longitude, heading, distance, bearing
  for (int i = 0; i < ULTRASONICS; i++) {
    LOG_PORT.print(Distance[i], 4);
    LOG_PORT.print(", ");
  }
  LOG_PORT.print(Fix.latitude(), 8);
  LOG_PORT.print(", ");
  LOG_PORT.print(Fix.longitude(), 8);
  LOG_PORT.print(", ");
  LOG_PORT.print(Heading, 6);
  LOG_PORT.print(", ");
  
  if (Latitude == 0 && Longitude == 0) {
    LOG_PORT.print(0);
    LOG_PORT.print(", ");
    LOG_PORT.print(0);
  }
  else {
    LOG_PORT.print(Fix.location.DistanceKm(destination), 6);
    LOG_PORT.print(", ");
    LOG_PORT.print(Fix.location.BearingToDegrees(destination), 6);
  }
  
  LOG_PORT.println();
}

void outputJSON() {
    for (int i = 0; i < ULTRASONICS; i++) {
      ultraJson[String(i)] = Distance[i];
    }
    gpsJson["latitude"] = Fix.latitude();
    gpsJson["longitude"] = Fix.longitude();
    gpsJson["heading"] = Heading;
    destJson["distance"] = Fix.location.DistanceMiles(destination);
    destJson["bearing"] = Fix.location.BearingToDegrees(destination);
    
    root.printTo(LOG_PORT);
    LOG_PORT.println();
}
