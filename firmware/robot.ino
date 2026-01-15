// Minimal AI Obstacle Avoidance Robot Firmware
// Assumes: HC-SR04 ultrasonic + differential drive

#include <Arduino.h>

const int trigPin = 9;
const int echoPin = 10;

float distance;
unsigned long lastTime;

enum State { FORWARD, TURN };
State state = FORWARD;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  return duration * 0.000343 / 2.0;
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  distance = readDistance();

  // Decision logic (minimal AI)
  if (distance < 0.3) state = TURN;
  else if (distance > 0.5) state = FORWARD;

  // Control outputs (abstracted)
  float v = (state == FORWARD) ? 0.2 : 0.0;
  float w = (state == TURN) ? 1.5 : 0.0;

  // Telemetry stream
  Serial.print(now / 1000.0); Serial.print(",");
  Serial.print(distance);     Serial.print(",");
  Serial.print(v);            Serial.print(",");
  Serial.println(w);

  delay(50);
}