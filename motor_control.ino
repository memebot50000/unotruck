#include <EnableInterrupt.h>

#define RC_NUM_CHANNELS  2
#define AVERAGE_SAMPLES  15

#define RC_CH1  0  // Steering
#define RC_CH2  1  // Throttle

#define RC_CH1_PIN  2
#define RC_CH2_PIN  3

// L298N motor control pins
#define ENA 9  // Enable A - Drive motor speed control
#define IN1 8  // Input 1 - Drive motor direction control
#define IN2 7  // Input 2 - Drive motor direction control
#define ENB 6  // Enable B - Steering motor speed control
#define IN3 12 // Input 3 - Steering motor direction control
#define IN4 13 // Input 4 - Steering motor direction control

#define DEADZONE_LOW  1450
#define DEADZONE_HIGH 1550
#define DEADZONE_CENTER 1500

#define MIN_MOTOR_SPEED 70
#define MAX_MOTOR_SPEED 179 // 70% of 255
#define STEERING_ACTIVATION_TIME 200 // 0.2 seconds in milliseconds

volatile unsigned long rc_rising_start[RC_NUM_CHANNELS];
volatile unsigned int rc_values[RC_NUM_CHANNELS];
unsigned int rc_average[RC_NUM_CHANNELS][AVERAGE_SAMPLES];
unsigned int rc_index[RC_NUM_CHANNELS];

unsigned long last_steering_activation = 0;
bool steering_activated = false;
bool steering_centered = true;

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

void setup() {
  Serial.begin(115200);

  pinMode(RC_CH1_PIN, INPUT);
  pinMode(RC_CH2_PIN, INPUT);

  enableInterrupt(RC_CH1_PIN, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_PIN, calc_ch2, CHANGE);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Increase PWM frequency for both motors
  TCCR1B = TCCR1B & B11111000 | B00000001; // Set PWM frequency for D9 and D10 to 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001; // Set PWM frequency for D3 and D11 to 31372.55 Hz

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
  for (int j = 0; j < AVERAGE_SAMPLES; j++) {
    sum += rc_average[channel][j];
  }
  return sum / AVERAGE_SAMPLES;
}

unsigned int apply_deadzone(unsigned int value) {
  if (value >= DEADZONE_LOW && value <= DEADZONE_HIGH) {
    return DEADZONE_CENTER;
  }
  return value;
}

void control_drive_motor(int speed) {
  int mappedSpeed = map(abs(speed - DEADZONE_CENTER), 0, 500, 0, MAX_MOTOR_SPEED);
  if (mappedSpeed < MIN_MOTOR_SPEED) {
    mappedSpeed = 0;
  }
  
  if (speed < DEADZONE_CENTER) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed > DEADZONE_CENTER) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  
  analogWrite(ENA, mappedSpeed);
}

void control_steering_motor(int position) {
  unsigned long current_time = millis();
  
  if (position == DEADZONE_CENTER) {
    steering_centered = true;
    steering_activated = false;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
    return;
  }
  
  if (steering_centered && !steering_activated) {
    steering_activated = true;
    last_steering_activation = current_time;
    steering_centered = false;
  }
  
  if (steering_activated && (current_time - last_steering_activation < STEERING_ACTIVATION_TIME)) {
    if (position > DEADZONE_CENTER) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, MAX_MOTOR_SPEED);
  } else {
    steering_activated = false;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void loop() {
  unsigned int ch1 = apply_deadzone(smooth_channel(RC_CH1));
  unsigned int ch2 = apply_deadzone(smooth_channel(RC_CH2));

  control_steering_motor(ch1);
  control_drive_motor(ch2);

  Serial.print(ch1); Serial.print(" ");
  Serial.print(ch2); Serial.print(" ");
  Serial.println(steering_activated);

  delay(20);
}
