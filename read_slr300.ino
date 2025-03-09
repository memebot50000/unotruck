#include <EnableInterrupt.h>

#define RC_NUM_CHANNELS  3
#define AVERAGE_SAMPLES  5

#define RC_CH1  0  // Steering
#define RC_CH2  1  // Throttle
#define RC_CH3  2  // Aux (3-position switch)

#define RC_CH1_PIN  2
#define RC_CH2_PIN  3
#define RC_CH3_PIN  4

#define DEADZONE_LOW  1475
#define DEADZONE_HIGH 1485
#define DEADZONE_CENTER 1480

// 3-position switch thresholds and values
#define AUX_LOW_THRESHOLD 1300
#define AUX_HIGH_THRESHOLD 1700
#define AUX_LOW_VALUE 1080
#define AUX_MID_VALUE 1480
#define AUX_HIGH_VALUE 1875

volatile unsigned long rc_rising_start[RC_NUM_CHANNELS];
volatile unsigned int rc_values[RC_NUM_CHANNELS];
unsigned int rc_average[RC_NUM_CHANNELS][AVERAGE_SAMPLES];
unsigned int rc_index[RC_NUM_CHANNELS];

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_rising_start[channel] = micros();
  } else {
    unsigned long now = micros();
    rc_values[channel] = (uint16_t)(now - rc_rising_start[channel]);
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_PIN); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_PIN); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_PIN); }

void setup() {
  Serial.begin(115200);

  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);
  pinMode(RC_CH3_PIN, INPUT);

  enableInterrupt(RC_CH1_PIN, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_PIN, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_PIN, calc_ch3, CHANGE);

  Serial.println("CH1 CH2 AUX");

  for (int i = 0; i < RC_NUM_CHANNELS; i++) {
    for (int j = 0; j < AVERAGE_SAMPLES; j++) {
      rc_average[i][j] = DEADZONE_CENTER;
    }
    rc_index[i] = 0;
  }
}

unsigned int smooth_channel(int channel) {
  rc_average[channel][rc_index[channel]] = rc_values[channel];
  rc_index[channel] = (rc_index[channel] + 1) % AVERAGE_SAMPLES;
  
  unsigned long sum = 0;
  for (int i = 0; i < AVERAGE_SAMPLES; i++) {
    sum += rc_average[channel][i];
  }
  return sum / AVERAGE_SAMPLES;
}

unsigned int apply_deadzone(unsigned int value) {
  if (value >= DEADZONE_LOW && value <= DEADZONE_HIGH) {
    return DEADZONE_CENTER;
  }
  return value;
}

unsigned int process_aux_switch(unsigned int value) {
  if (value < AUX_LOW_THRESHOLD) {
    return AUX_LOW_VALUE;
  } else if (value > AUX_HIGH_THRESHOLD) {
    return AUX_HIGH_VALUE;
  } else {
    return AUX_MID_VALUE;
  }
}

void loop() {
  unsigned int ch1 = apply_deadzone(smooth_channel(RC_CH1));
  unsigned int ch2 = apply_deadzone(smooth_channel(RC_CH2));
  unsigned int aux = process_aux_switch(smooth_channel(RC_CH3));

  Serial.print(ch1); Serial.print(" ");
  Serial.print(ch2); Serial.print(" ");
  Serial.println(aux);

  delay(20);
}
