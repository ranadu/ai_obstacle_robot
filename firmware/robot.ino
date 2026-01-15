// Minimal Obstacle Avoidance Firmware (hardware-assumed)
// Telemetry format matches simulator logs conceptually: t, distance, v, w, state

const int trigPin = 9;
const int echoPin = 10;

enum State { FORWARD, TURN };
State state = FORWARD;

const float STOP_D = 0.30;  // [m]
const float GO_D   = 0.55;  // [m]

unsigned long lastMs = 0;

float readDistanceM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long us = pulseIn(echoPin, HIGH, 25000); // timeout 25ms ~ 4.3m
  if (us == 0) return 999.0;               // no echo -> treat as far
  return (us * 0.000343f) / 2.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  lastMs = millis();
}

void loop() {
  unsigned long now = millis();
  float t = now / 1000.0f;

  float d = readDistanceM();

  // Two-state FSM with hysteresis
  if (state == FORWARD) {
    if (d < STOP_D) state = TURN;
  } else { // TURN
    if (d > GO_D) state = FORWARD;
  }

  // Control outputs (placeholder – your motor driver code will map these)
  float v = (state == FORWARD) ? 0.25f : 0.0f;
  float w = (state == TURN) ? 1.8f : 0.0f;

  // Telemetry: t,d,v,w,state
  Serial.print(t, 2); Serial.print(",");
  Serial.print(d, 3); Serial.print(",");
  Serial.print(v, 3); Serial.print(",");
  Serial.print(w, 3); Serial.print(",");
  Serial.println(state == FORWARD ? "FORWARD" : "TURN");

  delay(50); // ~20 Hz
}