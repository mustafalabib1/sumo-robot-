// === Motor Driver Pins (L298N) ===
#define ENA 11
#define FENA 10
#define IN1 13
#define IN2 12
#define IN3 8
#define IN4 7
#define ENB 6
#define FENB 5

// === Ultrasonic Sensor ===
#define TRIG_PIN 9
#define ECHO_PIN 4

// === IR Edge Sensors ===
#define IR_LEFT A0
#define IR_RIGHT A1
#define IR_FRONT A2
#define IR_BACK A3

// === Speed Settings ===
#define MAX_SPEED 255
#define SLOW_SPEED 130
#define TORNADO_SPEED 150

// === Thresholds ===
#define OPPONENT_DISTANCE 30  // cm
#define AVOID_DELAY 150       // ms

#define TRACK_WIDTH 0.2       // in meters
#define WHEEL_DIAMETER 0.065  // in meters
#define MOTOR_RPM 133         // RPM of the motor

float distance = 0, preDistance = 1000;

void DelayWithEdgeAvoidance(int delayTime);
double speedFromRPM(double rpm);
void moveCar(int leftSpeed, int rightSpeed);
void stopCar();
void rotateDegrees(float degrees, int speed);
void avoidEdge();
double readUltrasonic();
void attackOpponent();
void randomSearch();
void initPins();

void setup() {
  Serial.begin(9600);
  initPins();
  randomSeed(analogRead(A5));
}

void loop() {
  attackOpponent();
}

/// @brief Compute linear speed of a wheel from its RPM
/// @param rpm   Revolutions per minute
/// @return      Linear speed in meters per second (m/s)
double speedFromRPM(double rpm) {
  const double diameter_m = 0.065;           // 65 mm = 0.065 m
  double circumference = M_PI * diameter_m;  // π·D
  return (rpm * circumference) / 60.0;       // convert per minute → per second
}

void moveCar(int leftSpeed, int rightSpeed) {
  digitalWrite(IN3, leftSpeed > 0);
  digitalWrite(IN4, leftSpeed < 0);
  analogWrite(ENB, abs(leftSpeed));
  analogWrite(FENB, abs(leftSpeed * 133 / 170.0));

  digitalWrite(IN1, rightSpeed > 0);
  digitalWrite(IN2, rightSpeed < 0);
  analogWrite(ENA, abs(rightSpeed));
  analogWrite(FENA, abs(rightSpeed * 133 / 170.0));
}

void stopCar() {
  moveCar(0, 0);
}

void rotateDegrees(float degrees, int speed) {
  float arcLength = PI * TRACK_WIDTH * (degrees / 360.0);
  float wheelCircumference = PI * WHEEL_DIAMETER;
  float revolutions = arcLength / wheelCircumference;
  float timeSeconds = (revolutions * 60.0) / MOTOR_RPM;
  int timeMillis = (int)(timeSeconds * 1000);

  // Left wheel backward, right wheel forward for in-place rotation
  moveCar(-speed, speed);
  delay(timeMillis);
  moveCar(0, 0);  // Stop
}

double readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);  // timeout for safety
  if (duration == 0)
    return 999;  // if no echo
  return duration * 0.034 / 2.0;
}

void avoidEdge() {
  if (digitalRead(IR_LEFT)) {
    moveCar(SLOW_SPEED, -SLOW_SPEED);
    DelayWithEdgeAvoidance(AVOID_DELAY);
  } else if (digitalRead(IR_RIGHT)) {
    moveCar(-SLOW_SPEED, SLOW_SPEED);
    DelayWithEdgeAvoidance(AVOID_DELAY);
  } else if (digitalRead(IR_FRONT)) {
    moveCar(-SLOW_SPEED, -SLOW_SPEED);
    DelayWithEdgeAvoidance(AVOID_DELAY);
  } else if (digitalRead(IR_BACK)) {
    moveCar(MAX_SPEED, MAX_SPEED);
    DelayWithEdgeAvoidance(AVOID_DELAY);
  }
}
void DelayWithEdgeAvoidance(int delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
    avoidEdge();
    delay(10);  // Small delay to prevent blocking
  }
}
void attackOpponent() {
  preDistance = readUltrasonic();
  distance = preDistance;

  while (true) {
    distance = readUltrasonic();
    if (distance >= preDistance || distance > OPPONENT_DISTANCE || distance <= 0)
      break;

    moveCar(MAX_SPEED, MAX_SPEED);
    preDistance = distance;
    delay(20);
  }

  stopCar();
}

void randomSearch() {

  distance = readUltrasonic();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < OPPONENT_DISTANCE) {
    attackOpponent();
    return;
  }

  // Move forward until front IR detects edge
  while (digitalRead(IR_FRONT) == LOW) {
    moveCar(MAX_SPEED, MAX_SPEED);
    delay(5);  // Short delay for sensor stability
  }
  moveCar(-SLOW_SPEED, -SLOW_SPEED);
  delay(25);  // Move backward for 25ms
  stopCar();

  // Rotate randomly between 225° to 315°
  int angle = random(225, 316);
  rotateDegrees(angle, MAX_SPEED);
}

void initPins() {
  pinMode(ENA, OUTPUT);
  pinMode(FENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(FENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BACK, INPUT);
}
