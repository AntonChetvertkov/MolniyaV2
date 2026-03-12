#include <ESP32Servo.h>
#include <Wire.h>
#include <FastIMU.h>

#define PPM_PIN      34
#define AILERON_PIN  27
#define ELEVATOR_PIN 32
#define ESC_PIN      13
#define RUDDER_PIN   33
#define NUM_CHANNELS 8
#define SYNC_THRESHOLD 3000
#define IMU_ADDR     0x68

volatile uint16_t ppm[NUM_CHANNELS];
volatile uint8_t  channel   = 0;
volatile uint32_t last_time = 0;

Servo aileron, elevator, esc, rudder;

MPU6500 imu;
calData imu_cal = { 0 };
AccelData accel;

int  mode    = 0;
bool esc_arm = false;

float pitch_target = 0.0f;
float roll_target  = 0.0f;
bool  target_set   = false;

void IRAM_ATTR ppm_isr() {
    uint32_t now   = micros();
    uint32_t width = now - last_time;
    last_time = now;
    if (width > SYNC_THRESHOLD) channel = 0;
    else if (channel < NUM_CHANNELS) ppm[channel++] = width;
}

uint16_t get_channel(uint8_t ch) {
    uint16_t val;
    portDISABLE_INTERRUPTS();
    val = ppm[ch];
    portENABLE_INTERRUPTS();
    return val;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    int err = imu.init(imu_cal, IMU_ADDR);
    if (err != 0) Serial.printf("IMU init failed: %d\n", err);

    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppm_isr, RISING);

    aileron.attach(AILERON_PIN,   1000, 2000);
    elevator.attach(ELEVATOR_PIN, 1000, 2000);
    esc.attach(ESC_PIN,           1000, 2000);
    rudder.attach(RUDDER_PIN,     1000, 2000);
}

void loop() {
    uint16_t ch1 = get_channel(0);
    uint16_t ch2 = get_channel(1);
    uint16_t ch3 = get_channel(2);
    uint16_t ch4 = get_channel(3);
    uint16_t ch5 = get_channel(4);
    uint16_t ch6 = get_channel(5);
    uint16_t ch7 = get_channel(6);

    int prev_mode = mode;
    if      (ch7 >= 900  && ch7 < 1100) mode = 0;
    else if (ch7 >= 1400 && ch7 < 1600) mode = 1;
    else                                 mode = 2;

    if (prev_mode == 1 && mode != 1) target_set = false;

    if ((ch5 >= 900 && ch5 < 1100) || (ch6 >= 900 && ch6 < 1100)) {
        esc_arm = false;
        esc.writeMicroseconds(1000);
    } else if ((ch5 >= 1900 && ch5 < 2100) && (ch6 >= 1900 && ch6 < 2100)) {
        esc_arm = true;
    }

    imu.update();
    imu.getAccel(&accel);

    float pitch = atan2f(accel.accelX, accel.accelZ) * 57.2958f;
    float roll  = atan2f(accel.accelY, accel.accelZ) * 57.2958f;

    switch (mode) {
        case 0:
            if (ch1 >= 800 && ch1 <= 2200) aileron.writeMicroseconds(ch1);
            if (ch2 >= 800 && ch2 <= 2200) elevator.writeMicroseconds(ch2);
            if (ch3 >= 800 && ch3 <= 2200 && esc_arm) esc.writeMicroseconds(ch3);
            if (ch4 >= 800 && ch4 <= 2200) rudder.writeMicroseconds(ch4);
            break;

        case 1: {
            if (!target_set) {
                pitch_target = pitch;
                roll_target  = roll;
                target_set   = true;
            }

            int elev_stick = (int)ch2 - 1500;
            int ail_stick  = (int)ch1 - 1500;
            bool elev_override = abs(elev_stick) > 50;
            bool ail_override  = abs(ail_stick)  > 50;

            float pitch_err = pitch_target - pitch;
            int elev_us     = 1500 + constrain((int)(pitch_err * 10.0f), -400, 400);
            if (elev_override) {
                elev_us      = 1500 + elev_stick;
                pitch_target = pitch;
            }

            float roll_err = roll_target - roll;
            int ail_us     = 1500 + constrain((int)(roll_err * 10.0f), -400, 400);
            if (ail_override) {
                ail_us      = 1500 + ail_stick;
                roll_target = roll;
            }

            elev_us = constrain(elev_us, 1000, 2000);
            ail_us  = constrain(ail_us,  1000, 2000);

            aileron.writeMicroseconds(ail_us);
            elevator.writeMicroseconds(elev_us);
            if (ch3 >= 800 && ch3 <= 2200 && esc_arm) esc.writeMicroseconds(ch3);
            if (ch4 >= 800 && ch4 <= 2200) rudder.writeMicroseconds(ch4);

            Serial.printf("pitch=%.2f  perr=%.2f  elev=%d%s  |  roll=%.2f  rerr=%.2f  ail=%d%s\n",
                          pitch, pitch_err, elev_us, elev_override ? " [MAN]" : "",
                          roll,  roll_err,  ail_us,  ail_override  ? " [MAN]" : "");
            break;
        }
    }

    delay(20);
}