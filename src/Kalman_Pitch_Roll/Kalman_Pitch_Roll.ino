/*
 * Credit to https://github.com/TKJElectronics/KalmanFilter 
 * A lot of this code is from there since it works so well.
 */

#include <Wire.h>
#include<TinyGPS.h>
#include <Kalman.h>

#define MPU_ADDR 0x68
#define X_ACCEL_REG 0x3B
#define Y_ACCEL_REG 0x3D
#define Z_ACCEL_REG 0x3F
#define GYRO_X_REG 0x43
#define GYRO_Y_REG 0x45
#define GYRO_Z_REG 0x47

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double gyroXangle, gyroYangle; // Angle calculate using the gyro only

int16_t a_x,a_y,a_z;
int16_t gyro_x,gyro_y,gyro_z;//ADC readings are 16 bit
float x_offset,y_offset,z_offset;
float prev = 0;
int x = 0;
float pos_x = 0;
float acc_x = 0;
float acc_y = 0;
float acc_z = 0;
float velocity_x = 0;
float total_time = 0;

void update_data(){
  get_register_data(X_ACCEL_REG,&a_x);
  get_register_data(Y_ACCEL_REG,&a_y);
  get_register_data(Z_ACCEL_REG,&a_z);
  get_register_data(GYRO_X_REG,&gyro_x);
  get_register_data(GYRO_Y_REG,&gyro_y);
  get_register_data(GYRO_Z_REG,&gyro_z);
}


float condition_acceleration_data(int16_t* data){
  float my_data = ((float)*data/(16384.0+1227.0)) * 9.81;
  return my_data;
}

void init_MPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x08); //Disables temperature sensor. Can enable by resetting device.
  Wire.endTransmission(1);
}

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
  init_MPU();
  update_data();
  acc_x = ((float)a_x/(16384.0+1047.0)) * 9.81;
  acc_y = ((float)a_y/(16384.0-1143.0)) * 9.81;
  acc_z = ((float)a_z/(16384.0+1085.0)) * 9.81;
  double roll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
  double pitch = atan2(-acc_x,acc_z) * RAD_TO_DEG;
  double yaw = 0;
  /**
   * Set initial kalman parameters here.
   */
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


  //Update all data points.
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
    x=0;
  }
  
  
}

