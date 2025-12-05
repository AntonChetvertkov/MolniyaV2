#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "FastIMU.h"

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

//raw data arrays
float IMUdata[6];
float BAROdata[2];

void setup() {
  // AY SKVERED SI
  iic();

  //Cereal
  cereal(9600);

  //ImU
  imuSetup();

  //Baro
  baroSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  grabIMU();
  grabBaro();
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
  if (BARO1_INIT) BAROdata[0] = bmp1.readPressure();
  else {BAROdata[0] = -9999.0;}

  if (BARO2_INIT) BAROdata[1] = bmp2.readPressure();
  else {BAROdata[1] = -9999.0;}

}