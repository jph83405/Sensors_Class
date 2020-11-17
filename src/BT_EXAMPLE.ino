#include "BluetoothSerial.h"
  
BluetoothSerial SerialBT;
   
void setup()
{
  SerialBT.begin("ESP32test");
  Serial.begin(9600);
  delay(1000);
}
   
void loop()
{
  String sensorData;
  while (Serial.available()>0) {
    sensorData = Serial.readString();
    Serial.println(sensorData);
    SerialBT.println(sensorData);
  }
}

