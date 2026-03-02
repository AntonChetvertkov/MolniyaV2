#include <ESP32Servo.h>


#define PPM_PIN 34
#define AILERON_PIN 27
#define ELEVATOR_PIN 32
#define ESC_PIN 13
#define RUDDER_PIN 33
#define NUM_CHANNELS 8
#define SYNC_THRESHOLD 3000

volatile uint16_t ppm[NUM_CHANNELS];
volatile uint8_t channel = 0;
volatile uint32_t last_time = 0;

Servo aileron;
Servo elevator;
Servo esc;
Servo rudder;

int mode = 0; //0 - manual; 1 - hold pos
bool esc_arm = 0;

void IRAM_ATTR ppm_isr() {
    uint32_t now = micros();
    uint32_t width = now - last_time;
    last_time = now;

    if (width > SYNC_THRESHOLD) {
        channel = 0;
    } else if (channel < NUM_CHANNELS) {
        ppm[channel++] = width;
    }
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
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppm_isr, RISING);
    aileron.attach(AILERON_PIN, 1000, 2000);
    elevator.attach(ELEVATOR_PIN, 1000, 2000);
    esc.attach(ESC_PIN, 1000, 2000);
    rudder.attach(RUDDER_PIN, 1000, 2000);
}

void loop() {
  //upd the chan vals
  uint16_t ch1 = get_channel(0);
  uint16_t ch2 = get_channel(1);
  uint16_t ch3 = get_channel(2);
  uint16_t ch4 = get_channel(3);
  uint16_t ch5 = get_channel(4);
  uint16_t ch6 = get_channel(5);
  uint16_t ch7 = get_channel(6);

  //modes
  if (ch7 >= 900 && ch7 < 1100) mode = 0;
  else if (ch7 >= 1400 && ch7 < 1600) mode = 1;
  else mode = 2;

  //throttle arming
  if ((ch5 >= 900 && ch5 < 1100) || (ch6 >= 900 && ch6 < 1100)) {esc_arm = 0;  esc.writeMicroseconds(1000);}
  else if ((ch5 >= 1900 && ch5 < 2100) && (ch6 >= 1900 && ch6 < 2100)) {esc_arm = 1;}
  

  switch(mode) {
    case 0: // manual mode
      if (ch1 >= 800 && ch1 <= 2200) {
        aileron.writeMicroseconds(ch1);
      }
      if (ch2 >= 800 && ch2 <=2200){
        elevator.writeMicroseconds(ch2);
      }
      if ((ch3 >= 800 && ch3 <= 2200) && esc_arm){
        esc.writeMicroseconds(ch3);
      }
      if (ch4 >= 800 &&ch4 <=2200){
        rudder.writeMicroseconds(ch4);
      }
      break;
  }
  delay(20);
}