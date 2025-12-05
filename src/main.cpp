#include <Wire.h>
#include <vector>
#include <Adafruit_BMP280.h>
#include "FastIMU.h"
using std::vector;

//statusis
bool IMU_INIT = false;
bool BARO1_INIT = false;
bool BARO2_INIT = false;
bool GNSS_INIT = false;
bool TELEM_INIT = false;

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
vector <float> grabIMU();
void baroSetup();
vector <float> grabBaro();


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
  vector <float> IMUdata = grabIMU();
  vector <float> BAROdata = grabBaro();
}

void iic(){
  Wire.begin();
  Wire.setClock(400000);
}

void cereal(int baudRate){
  Serial.begin(baudRate);
}

void imuSetup(){
  int imuErr = IMU.init(calib, IMU_ADDRESS);
  if (imuErr != 0){Serial.println("Failed to init IMU. "+String(imuErr));}
  else {
    IMU_INIT = true;
    Serial.println("Begining Calibration) Standby");
    IMU.calibrateAccelGyro(&calib);
    IMU.init(calib, IMU_ADDRESS);
  }
}

vector <float> grabIMU(){
  vector <float> data;
  if (IMU_INIT){
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);
    data = {
      accelData.accelX,
      accelData.accelY,
      accelData.accelZ,
      gyroData.gyroX,
      gyroData.gyroY,
      gyroData.gyroZ,
    };
  }
  else{
    for (int i = 0; i < 6; i++){
      data.push_back(-9999.0);
    }
  }
  return data;
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
  if (!BARO2_INIT) Serial.println("Baro on "+String(baro_addresses[1])+" failed");
  else{
    bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  }
}

vector <float> grabBaro(){
  vector <float> data;
  if (BARO1_INIT) data.push_back(bmp1.readPressure());
  if (BARO2_INIT) data.push_back(bmp2.readPressure());
  if (!BARO1_INIT && !BARO2_INIT){
    data.push_back(-9999.0);
    data.push_back(-9999.0);
  }
  return data;
}