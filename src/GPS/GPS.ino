#include<TinyGPS.h>
#include <SoftwareSerial.h>


SoftwareSerial gpsSerial(3,4);
TinyGPS gps;
float latt,lon;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  

}

void loop() {
   while (gpsSerial.available() > 0){
    if(gps.encode(gpsSerial.read())){
      gps.f_get_position(&latt, &lon);
    }
   }
   Serial.print("Latitude: ");
   Serial.println(latt, 6);
   Serial.print("Longitude: ");
   Serial.println(lon, 6);
   delay(1000);
  
}

/*
void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    Serial.println("Location: Not Available");
  }


  Serial.println();
  Serial.println();
  delay(1000);
}
*/
