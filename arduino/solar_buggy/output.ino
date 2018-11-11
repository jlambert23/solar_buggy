// Json output objects
DynamicJsonBuffer jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& gpsJson = root.createNestedObject("gps");
JsonObject& destJson = gpsJson.createNestedObject("waypoint");
JsonObject& ultraJson = root.createNestedObject("ultrasonic");

void outputCSV() {
  // ultrasonic_0, ultrasonic_1, ultrasonic_2, ultrasonic_3, longitude, latitude, heading, distance, bearing
  for (int i = 0; i < ULTRASONICS; i++) {
    OUTPUT_SERIAL.print(Distance[i]);
    OUTPUT_SERIAL.print(", ");
  }
  OUTPUT_SERIAL.print(Fix.longitude());
  OUTPUT_SERIAL.print(", ");
  OUTPUT_SERIAL.print(Fix.latitude());
  OUTPUT_SERIAL.print(", ");
  OUTPUT_SERIAL.print(Heading);
  OUTPUT_SERIAL.print(", ");
  OUTPUT_SERIAL.print(Fix.location.DistanceMiles(destination));
  OUTPUT_SERIAL.print(", ");
  OUTPUT_SERIAL.print(Fix.location.BearingToDegrees(destination));

  OUTPUT_SERIAL.println();
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
    
    root.printTo(OUTPUT_SERIAL);
    OUTPUT_SERIAL.println();
}
