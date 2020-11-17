/*
 * Author: Justin Heimerl
 * 11/17/20
 * Purpose: Code integrates various sensors to measure power draw of an RC car as a function of path traveled down a hill.
 * Data sent out via serial port to ESP32 module, then sent out via bluetooth.
 */

#include<TinyGPS.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Kalman.h>

#define MPU_ADDR 0x68
#define X_ACCEL_REG 0x3B
#define Y_ACCEL_REG 0x3D
#define Z_ACCEL_REG 0x3F
#define GYRO_X_REG 0x43
#define GYRO_Y_REG 0x45
#define GYRO_Z_REG 0x47

const int vPIN = A4;
float vOUT = 0.0;
float vIN = 0.0;
float R1 = 1300.0;       //Resistor 1 value in ohms
float R2 = 5100.0;       //Resistor 2 value in ohms
int voltage_reading = 0;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

SoftwareSerial gpsSerial(3,4);//Create software serial interface for comms with GPS
TinyGPS gps;//Tiny GPS object to parse GPS data.
float latt,lon;

//Accel and gyro data
int16_t a_x,a_y,a_z;
int16_t gyro_x,gyro_y,gyro_z;//ADC readings are 16 bit
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
float x_offset,y_offset,z_offset;
float prev = 0;
int x = 0;
float acc_x = 0;
float acc_y = 0;
float acc_z = 0;
float total_time = 0;

//Function to read MPU registers and update accel/gyro variables.
void update_data(){
  get_register_data(X_ACCEL_REG,&a_x);
  get_register_data(Y_ACCEL_REG,&a_y);
  get_register_data(Z_ACCEL_REG,&a_z);
  get_register_data(GYRO_X_REG,&gyro_x);
  get_register_data(GYRO_Y_REG,&gyro_y);
  get_register_data(GYRO_Z_REG,&gyro_z);
}

//Initializes MPU, turns temp sensor off for now.
void init_MPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x08); //Disables temperature sensor. Can enable by resetting device.
  Wire.endTransmission(1);
}

//Gets MPU register data from the reg, stores data in data variable.
void get_register_data(uint16_t reg,int16_t* data){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(0);//Don't end transmission yet.
  Wire.requestFrom(MPU_ADDR,2,1); //Store data inside buffer in wire library
  byte MSB = Wire.read();//MPU has high register first,then low register
  byte LSB = Wire.read();
  *data = ((MSB << 8) | LSB); //16bits that represent our data from the MPU.
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  //Initialize all MPU parameters.
  init_MPU();
  update_data();
  acc_x = ((float)a_x/(16384.0+1046.0)) * 9.81;
  acc_y = ((float)a_y/(16384.0-1149.0)) * 9.81;
  acc_z = ((float)a_z/(16384.0+1092.0)) * 9.81;
  double roll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
  double pitch = atan2(-acc_x,acc_z) * RAD_TO_DEG;
  double yaw = 0;
 
   // Set initial kalman parameters here.
   kalmanX.setAngle(roll);
   kalmanY.setAngle(pitch);
   gyro_x = pitch;
   gyro_y = roll;
  

}

void loop() {
  //Take care of timing the system.
  unsigned long now = millis();
  float dt = ((float)now - (float)prev)/1000.0f;
  total_time = total_time + dt;
  prev = now;
  //Update all data points for MPU.
  update_data();
  //Condition acceleration data
  acc_x = ((float)a_x/(16384.0+1072.0)) * 9.81;
  acc_y = ((float)a_y/(16384.0-1164.0)) * 9.81;
  acc_z = ((float)a_z/(16384.0+1099.0)) * 9.81;
  double roll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
  double pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;


  
  double gyroXrate = gyro_x / 131.0; // Convert to deg/s
  double gyroYrate = gyro_y / 131.0; // Convert to deg/s

  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;


    // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

   while (gpsSerial.available() > 0){
      if(gps.encode(gpsSerial.read())){
      gps.f_get_position(&latt, &lon);
    }
   }

   float amp = analogRead(A3);
   amp = amp - 510; 
   amp = (amp * 5.0) / 1024.0; 
   amp = amp * 10;
   voltage_reading = analogRead(vPIN);
   vOUT = (voltage_reading * 5.0) / 1024.0;
   vIN = vOUT / ( R2 / (R1 + R2) );
   voltage_reading = analogRead(vPIN);
   float watt = amp * vIN; 
  
  x++;
  if(x == 1000){
    Serial.print("X acc = ");
    Serial.println(acc_x);
    Serial.print("Y acc = ");
    Serial.println(acc_y);
    Serial.print("Z acc = ");
    Serial.println(acc_z);
    Serial.print("Kalman Pitch = ");
    Serial.println(kalAngleY);
    Serial.print("Kalman Roll = ");
    Serial.println(kalAngleX);
    Serial.print("Time = ");
    Serial.print(total_time);
    Serial.println(" seconds");
    Serial.print("Latitude: ");
    Serial.println(latt, 6);
    Serial.print("Longitude: ");
    Serial.println(lon, 6);
    Serial.print("Voltage: ");
    Serial.print(vIN);
    Serial.println("V");
    Serial.print("W: ");
    Serial.println(watt,1);
    delay(500);
    x=0;
  }
}


