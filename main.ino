#include <Servo.h>
#include <TinyGPS++.h>

Servo panMotor;  
Servo tiltMotor; 

const int panPin = 9;
const int tiltPin = 10;

double base_lat = 0.0, base_lon = 0.0, base_alt = 0.0;  // set your base station GPS data
double uav_lat, uav_lon, uav_alt;

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  panMotor.attach(panPin); 
  tiltMotor.attach(tiltPin); 
  
  panMotor.write(90);  // set initial position
  tiltMotor.write(90);
}

void loop() {
  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      if (gps.location.isValid()) {
        uav_lon = gps.location.lng();
        uav_lat = gps.location.lat();
        uav_alt = gps.altitude.meters();
      }
    }
  }
  
  if (gps.location.isUpdated()) {
    // Calculate the azimuth and elevation
    int azimuth = calculateAzimuth(uav_lat, uav_lon, base_lat, base_lon);
    int elevation = calculateElevation(uav_lat, uav_lon, uav_alt, base_lat, base_lon);
    
    // Update the position of the servos
    panMotor.write(azimuth);
    tiltMotor.write(elevation);
  }
}

int calculateAzimuth(double lat1, double long1, double lat2, double long2) {
  // convert to radians
  double lat1rad = lat1 * DEG_TO_RAD;
  double long1rad = long1 * DEG_TO_RAD;
  double lat2rad = lat2 * DEG_TO_RAD;
  double long2rad = long2 * DEG_TO_RAD;

  double deltaLong = long2rad - long1rad;

  double y = sin(deltaLong) * cos(lat2rad);
  double x = cos(lat1rad) * sin(lat2rad) - sin(lat1rad) * cos(lat2rad) * cos(deltaLong);
  double bearing = atan2(y, x);

  // convert from radians to degrees
  double bearingDegrees = bearing * RAD_TO_DEG;

  // adjust to 0-360 degrees range
  if(bearingDegrees < 0)
    bearingDegrees = 360 + bearingDegrees;

  return (int)bearingDegrees;
}

int calculateElevation(double lat1, double long1, double alt1, double lat2, double long2, double alt2) {
  // Calculate distance between points (in km)
  double distance = TinyGPSPlus::distanceBetween(lat1, long1, lat2, long2) / 1000;
  double delta_alt = alt2 - alt1;

  // Calculate elevation angle
  double elevation_angle = atan2(delta_alt, (distance*1000)) * RAD_TO_DEG;

  return (int)elevation_angle;
}
