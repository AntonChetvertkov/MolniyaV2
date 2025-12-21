#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "FastIMU.h"
#include <GyverFilters.h>
#include <vector>
#include <HardwareSerial.h>

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

//lora config
#define LORA_TX 17
#define LORA_RX 16
#define LORA_M0 4
#define LORA_M1 5
#define LORA_AUX 18
HardwareSerial LoraSerial(2);

//func prototype
void iic();
void cereal(int);
void imuSetup();
void loraSetup();
void grabIMU();
void baroSetup();
void grabBaro();
float PressToAlt(float pressure);
void attitude_begin();
void attitude_update(float ax, float ay, float az, float gx, float gy, float gz, float dt);
void print_filtered_data();
void encode_data();
void decode_data();
void update_lora();

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
float altitude = 0;
float velocity = 0;
float vertical_velocity = 0;
float vertical_velocity_raw = 0;
float pitch_angle = 127;
float roll_angle = 127;
float altitude_raw = 127;
double gnss_latitude = 0;
double gnss_longitude = 0;
float gnss_hdop = 0;
byte gnss_sat_number;
float battery_voltage = 0;
byte vehicle_state = 0;
byte autopilot_state = 0;
byte command = 0;
uint32_t checksum = 0;
byte start_byte = 45;
byte COBS = 1;

byte command_recv = 0;

byte packet[26];
byte packet_recv[26];


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

  //lorka setup
  loraSetup();

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
    encode_data(255);
    update_lora();
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

void encode_data(byte target = 255){
  packet[0] = start_byte;
  packet[2] = target;
  packet[3] = (byte)((int8_t)vertical_velocity_raw + 127);
  packet[4] = (byte)((int8_t)vertical_velocity+127);
  packet[5] = velocity;
  packet[6] = (byte)((int8_t)pitch_angle+127);
  packet[7] = (byte)((int8_t)roll_angle+127);
  packet[8] = altitude_raw;
  packet[9] = altitude;

  //gnss manipulating

  int32_t latitude_mdeg = gnss_latitude * 1000000;
  byte latitude_mdeg_0 = latitude_mdeg & 0xFF;
  byte latitude_mdeg_1 = (latitude_mdeg >> 8) & 0xFF;
  byte latitude_mdeg_2 = (latitude_mdeg >> 16) & 0xFF;
  byte latitude_mdeg_3 = (latitude_mdeg >> 24) & 0xFF;
  int32_t longitude_mdeg = gnss_longitude*1000000;
  byte longitude_mdeg_0 = longitude_mdeg & 0xFF;
  byte longitude_mdeg_1 = (longitude_mdeg >> 8) & 0xFF;
  byte longitude_mdeg_2 = (longitude_mdeg >> 16) & 0xFF;
  byte longitude_mdeg_3 = (longitude_mdeg >> 24) & 0xFF;

  packet[10] = latitude_mdeg_0;
  packet[11] = latitude_mdeg_1;
  packet[12] = latitude_mdeg_2;
  packet[13] = latitude_mdeg_3;
  packet[14] = longitude_mdeg_0;
  packet[15] = longitude_mdeg_1;
  packet[16] = longitude_mdeg_2;
  packet[17] = longitude_mdeg_3;
  packet[18] = gnss_hdop;
  packet[19] = gnss_sat_number;
  packet[20] = (byte)(battery_voltage*10);
  packet[21] = vehicle_state;
  packet[22] = autopilot_state;
  packet[23] = command;
  
  //fletcher16 checksum

  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  for (int i = 2; i < 24; i++){
    sum1 = (sum1 + packet[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  uint16_t checksum = (sum2 << 8) | sum1;
  packet[24] = checksum & 0xFF;
  packet[25] = (checksum >> 8) & 0xFF;

  //COBS

  std::vector<uint8_t> start_byte_occurances;
  for (int i = 2; i < 26; i++){
    if (packet[i] == start_byte) start_byte_occurances.push_back((uint8_t)i);
  }
  
  if (start_byte_occurances.size() == 0) {
    packet[1] = 255;
  } else {
    packet[1] = start_byte_occurances[0];
    
    for (int j = 0; j < start_byte_occurances.size() - 1; j++){
      packet[start_byte_occurances[j]] = start_byte_occurances[j + 1];
    }
    
    packet[start_byte_occurances[start_byte_occurances.size() - 1]] = 255;
  }
}

void decode_data() {
  if (packet_recv[0] != start_byte) return;
  
  std::vector<uint8_t> cobs_chain;
  uint8_t next_pos = packet_recv[1];
  
  while (next_pos != 255 && next_pos < 26) {
    cobs_chain.push_back(next_pos);
    next_pos = packet_recv[next_pos];
  }
  
  for (uint8_t pos : cobs_chain) {
    packet_recv[pos] = start_byte;
  }
  
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  for (int i = 2; i < 24; i++) {
    sum1 = (sum1 + packet_recv[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  uint16_t calculated_checksum = (sum2 << 8) | sum1;
  uint16_t received_checksum = packet_recv[25] << 8 | packet_recv[24];
  
  if (calculated_checksum != received_checksum) {
    command_recv = 17;
    return;
  }
  
  command_recv = packet_recv[23];
}

void loraSetup() {
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  
  LoraSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
}

void update_lora(){ 
  if (LoraSerial.available() >= 26) {
    LoraSerial.readBytes(packet_recv, 26);
    decode_data();
  }
  if (command_recv == 17){
    while(digitalRead(LORA_AUX) == LOW) delay(0.5);
    LoraSerial.write(packet, 26);
    command_recv = 0;
  }
}