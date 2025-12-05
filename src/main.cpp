#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "FastIMU.h"
#include <GyverFilters.h>

//statusis
bool IMU_INIT = false;
bool BARO1_INIT = false;
bool BARO2_INIT = false;
bool GNSS_INIT = false;
bool TELEM_INIT = false;

//AY2SI
#define SDA 21
#define SCL 22

///imu config
#define IMU_ADDRESS 0x68
MPU6500 IMU;
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;

///baro config
Adafruit_BMP280 bmp1;
Adafruit_BMP280 bmp2;
byte baro_addresses[2] = {0x76, 0x77};


//func prototype
void iic();
void cereal(int);
void imuSetup();
void grabIMU();
void baroSetup();
void grabBaro();
float PressToAlt(float pressure);
void attitude_begin();
void attitude_update(float ax, float ay, float az, float gx, float gy, float gz, float dt);
void print_filtered_data();

//raw data arrays
float IMUdata[6];
float BAROdata[3];

//filters
GKalman filterAlt(40, 0.5);
GKalman filterVel(20, 0.8);

GFilterRA filterAccelX(3);
GFilterRA filterAccelY(3);
GFilterRA filterAccelZ(3);

GFilterRA filterGyroX(3);
GFilterRA filterGyroY(3);
GFilterRA filterGyroZ(3);

struct AttitudeFilter {
    float roll;
    float pitch;
    float yaw;
    bool initted;
};

AttitudeFilter attitude;

unsigned long last_time;
float dt;
float ground_pressure;
float altitude;
float velocity;
float last_altitude;

void setup() {
  // AY SKVERED SI
  iic();

  //Cereal
  cereal(115200);

  //ImU
  imuSetup();

  //Baro
  baroSetup();

  attitude_begin();

  if (BARO1_INIT) ground_pressure = bmp1.readPressure();
  else if (BARO2_INIT) ground_pressure = bmp2.readPressure();
  else ground_pressure = 101325.0;

  altitude = 0.0;
  velocity = 0.0;
  last_altitude = 0.0;
  last_time = micros();
}

void loop() {
  unsigned long current_time = micros();
  dt = (current_time - last_time)/1000000.0;
  last_time = current_time;

  grabIMU();
  grabBaro();

  if (BARO1_INIT || BARO2_INIT) {
    float measured_alt = PressToAlt(BAROdata[2]);
    altitude = filterAlt.filtered(measured_alt);

    float unfiltered_beer_no_velocity = (altitude - last_altitude)/dt;
    velocity = filterVel.filtered(unfiltered_beer_no_velocity); 
    last_altitude = altitude;
  }
  if (IMU_INIT) {
        float ax = filterAccelX.filtered(IMUdata[0]);
        float ay = filterAccelY.filtered(IMUdata[1]);
        float az = filterAccelZ.filtered(IMUdata[2]);
        
        float gx = filterGyroX.filtered(IMUdata[3]);
        float gy = filterGyroY.filtered(IMUdata[4]);
        float gz = filterGyroZ.filtered(IMUdata[5]);
        
        attitude_update(ax, ay, az, gx, gy, gz, dt);
    }
    print_filtered_data();
    
    delay(10);
}

void iic(){
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);

}

void cereal(int baudRate){
  Serial.begin(baudRate);
}

void imuSetup(){
  int imuErr = IMU.init(calib, IMU_ADDRESS);
  if (imuErr != 0){
    Serial.print("Failed to init IMU. ");
    Serial.println(imuErr);
  }
  else {
    IMU_INIT = true;
    Serial.println("Begining Calibration) Standby");
    IMU.calibrateAccelGyro(&calib);
    IMU.init(calib, IMU_ADDRESS);
  }
}

void grabIMU() {
  if (IMU_INIT) {
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);
    IMUdata[0] = accelData.accelX;
    IMUdata[1] = accelData.accelY;
    IMUdata[2] = accelData.accelZ;
    IMUdata[3] = gyroData.gyroX;
    IMUdata[4] = gyroData.gyroY;
    IMUdata[5] = gyroData.gyroZ;
  } else {
    for (int i = 0; i < 6; i++) IMUdata[i] = -9999.0;
  }
}

void baroSetup(){
  BARO1_INIT = bmp1.begin(baro_addresses[0]);
  if (!BARO1_INIT) Serial.println("Baro on "+String(baro_addresses[0])+" failed");
  else{
    bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  }
  BARO2_INIT = bmp2.begin(baro_addresses[1]);
  delay(5);
  if (!BARO2_INIT) Serial.println("Baro on "+String(baro_addresses[1])+" failed");
  else{
    bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  }
}

void grabBaro(){
  if (BARO1_INIT) {
    BAROdata[0] = bmp1.readPressure();
  }
  else {BAROdata[0] = 0.0;}

  if (BARO2_INIT) {
    BAROdata[1] = bmp2.readPressure();
  }
  else {BAROdata[1] = 0.0;}
  
  if (BARO1_INIT && BARO2_INIT) {BAROdata[2] = (BAROdata[0] + BAROdata[1])/2.0;}
  else if ((!BARO1_INIT && BARO2_INIT) || (BARO1_INIT && !BARO2_INIT)){BAROdata[2] = (BAROdata[0] + BAROdata[1])/1.0;}
  else {BAROdata[2] = 0.0;}
}

float PressToAlt(float pressure) {
    return 44330.0 * (1.0 - pow(pressure / ground_pressure, 0.1903));
}

void attitude_begin() {
    attitude.roll = 0.0;
    attitude.pitch = 0.0;
    attitude.yaw = 0.0;
    attitude.initted = false;
}

void attitude_update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    const float alpha = 0.98;
    
    if (!attitude.initted) {
        attitude.roll = atan2(ay, az) * 57.2958;
        attitude.pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
        attitude.yaw = 0.0;
        attitude.initted = true;
        return;
    }
    
    attitude.roll += gx * dt;
    attitude.pitch += gy * dt;
    attitude.yaw += gz * dt;
    
    float accel_roll = atan2(ay, az) * 57.2958;
    float accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
    
    attitude.roll = alpha * attitude.roll + (1.0 - alpha) * accel_roll;
    attitude.pitch = alpha * attitude.pitch + (1.0 - alpha) * accel_pitch;
}

void print_filtered_data() {
    Serial.print("Alt: ");
    Serial.print(altitude, 2);
    Serial.print(" m, Vel: ");
    Serial.print(velocity, 2);
    Serial.print(" m/s, Roll: ");
    Serial.print(attitude.roll, 1);
    Serial.print("°, Pitch: ");
    Serial.print(attitude.pitch, 1);
    Serial.print("°, Yaw: ");
    Serial.print(attitude.yaw, 1);
    Serial.println("°");
}