
#include <Adafruit_GPS.h>
#include <math.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define UPDATE_RATE 1000
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Define the maximum course correction angle
#define MAX_CORRECTION 180

uint32_t timer = millis();

// Constants for the GPS calculations
const float R = 6371000.0; // Earth's mean radius in meters
const float METERS_TO_NAUTICAL_MILES = 0.000539957; // Conversion factor: 1 meter = 0.000539957 nautical miles

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop()
{  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }


  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > UPDATE_RATE) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);

    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); 
    if (GPS.fix) {
      if (GPS.fix == 1) {
        Serial.print("GPS");
      } else {
        Serial.print("DGPS");
      }
      Serial.print(" Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna: "); Serial.println(GPS.antenna);
      Serial.print("Speed (n): "); Serial.println(GPS.speed);

      Serial.print("Location: "); 
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 6);

      // Get the current position
      float lat = GPS.latitudeDegrees;
      float lon = GPS.longitudeDegrees;

      // Get the waypoint
      float waypoint_lat = 37.7749;
      float waypoint_lon = -121.855179;

      // Calculate the cross track error
      float distance = getDistance(lat, lon, waypoint_lat, waypoint_lon);
      float bearing = getBearing(lat, lon, waypoint_lat, waypoint_lon);
      float course = GPS.angle;
      float course_correction = greatCircleCourseCorrection(bearing , course);


    // Print the results
    Serial.print("Waypoint: ");
    Serial.print(waypoint_lat, 6);
    Serial.print(", ");
    Serial.println(waypoint_lon, 6);
    Serial.print("Distance: ");
    Serial.println(distance);


    Serial.print("Bearing: ");
    Serial.println(bearing, 2);


    Serial.print("Course: ");
    Serial.println(course);

    Serial.print("Course Correction: ");
    Serial.print(course_correction, 2);
    Serial.print(" degrees ");
    Serial.println((course_correction > 0) ? "R" : "L");     
    } else {
      Serial.println("None");
    }
  }
}

float toRadians(float degrees) {
  return degrees * PI / 180.0;
}

float toDegrees(float radians) {
  return radians * 180.0 / PI;
}

float greatCircleCourseCorrection(float desiredCourse, float actualHeading) {
  float desiredCourseRad = toRadians(desiredCourse);
  float actualHeadingRad = toRadians(actualHeading);
  float courseCorrectionRad = atan2(sin(desiredCourseRad - actualHeadingRad), cos(actualHeadingRad));
  float courseCorrectionDeg = toDegrees(courseCorrectionRad);
  return courseCorrectionDeg;
}


// Calculates the distance between two points on the Earth's surface
float getDistance(float lat1, float lon1, float lat2, float lon2) {
  float dlat = toRadians(lat2 - lat1);
  float dlon = toRadians(lon2 - lon1);
  float a = sin(dlat/2) * sin(dlat/2) + cos(toRadians(lat1)) * cos(toRadians(lat2)) * sin(dlon/2) * sin(dlon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c * METERS_TO_NAUTICAL_MILES;
}

// Calculates the initial bearing from point 1 to point 2
float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float y = sin(toRadians(lon2-lon1)) * cos(toRadians(lat2));
  float x = cos(toRadians(lat1))*sin(toRadians(lat2)) - sin(toRadians(lat1))*cos(toRadians(lat2))*cos(toRadians(lon2-lon1));
  float bearing = toDegrees(atan2(y, x));

  if (bearing < 0) {
    bearing += 360.0;
  }
  
  if ((roundf(bearing * 100)/100) == 360.00) {
    bearing = 0.00;
  }
  return bearing;
}

// Function to calculate the bearing between two points
double calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float bearing = atan2(y, x);
  bearing = degrees(bearing);
  bearing = fmod((bearing + 360), 360);
  return bearing;
}

