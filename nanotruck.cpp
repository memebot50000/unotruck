#include <EnableInterrupt.h>

#define THROTTLE_IN_PIN 6
#define STEERING_IN_PIN 7
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

#define ENA 9
#define IN1 2
#define IN2 3
#define ENB 10
#define IN3 4
#define IN4 5

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define NEUTRAL_SIGNAL 1500
#define DEAD_ZONE 20

volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

void setup() {
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  enableInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
  enableInterrupt(STEERING_IN_PIN, calcSteering, CHANGE);
}

void loop() {
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint8_t bUpdateFlags;

  if(bUpdateFlagsShared) {
    noInterrupts();
    bUpdateFlags = bUpdateFlagsShared;
    
    if(bUpdateFlags & THROTTLE_FLAG) {
      unThrottleIn = unThrottleInShared;
    }
    
    if(bUpdateFlags & STEERING_FLAG) {
      unSteeringIn = unSteeringInShared;
    }
    
    bUpdateFlagsShared = 0;
    interrupts();
  }

  if(bUpdateFlags & THROTTLE_FLAG) {
    int throttleSpeed = map(unThrottleIn, MIN_SIGNAL, MAX_SIGNAL, -255, 255);
    setDriveMotor(throttleSpeed);
  }

  if(bUpdateFlags & STEERING_FLAG) {
    int steeringSpeed = map(unSteeringIn, MIN_SIGNAL, MAX_SIGNAL, -255, 255);
    setSteeringMotor(steeringSpeed);
  }

  bUpdateFlags = 0;
}

void setDriveMotor(int speed) {
  if (abs(speed) < DEAD_ZONE) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  }
}

void setSteeringMotor(int speed) {
  if (abs(speed) < DEAD_ZONE) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  }
}

void calcThrottle() {
  if(digitalRead(THROTTLE_IN_PIN) == HIGH) {
    ulThrottleStart = micros();
  } else {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering() {
  if(digitalRead(STEERING_IN_PIN) == HIGH) {
    ulSteeringStart = micros();
  } else {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}
