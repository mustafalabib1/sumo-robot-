// === Motor Driver Pins (L298N) ===
#define ENA 10
#define IN1 13
#define IN2 12
#define IN3 8
#define IN4 7
#define ENB 5
// === Ultrasonic Sensor ===
#define TRIG_PIN 9
#define ECHO_CENTER 4
#define ECHO_RIGHT 11
#define ECHO_LEFT 3

// === IR Edge Sensors ===
#define IR_LEFT A0
#define IR_FRONT2 A1
#define IR_FRONT1 A2
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
double readUltrasonic(int echoPin);
void attackOpponent();
void randomSearch();
void initPins();

void setup() {
  Serial.begin(9600);
  initPins();
  randomSeed(analogRead(A5));
  // delay(10000);
}

void loop() {
  if (digitalRead(IR_FRONT1)==LOW) {
    moveCar(-MAX_SPEED, -MAX_SPEED);
    delay(100);
    rotateDegrees(90);
    rotateDegreesWithUltrasonic(random(0, 90));
  } else if (digitalRead(IR_FRONT2)==LOW) {
    moveCar(-MAX_SPEED, -MAX_SPEED);
    delay(100);
    rotateDegrees(-90);
    rotateDegreesWithUltrasonic(-random(0, 90));
  } else {
    double distLeft = readUltrasonic(ECHO_LEFT);
    double distRight = readUltrasonic(ECHO_RIGHT);
    if (distLeft < 20)
      rotateDegreesWithUltrasonic(90);
    else if (distRight < 20)
      rotateDegreesWithUltrasonic(-90);
    else
      moveCar(MAX_SPEED, MAX_SPEED);
  }
}

void moveCar(int leftSpeed, int rightSpeed) {
  digitalWrite(IN3, leftSpeed > 0);
  digitalWrite(IN4, leftSpeed < 0);
  analogWrite(ENB, abs(leftSpeed));

  digitalWrite(IN1, rightSpeed > 0);
  digitalWrite(IN2, rightSpeed < 0);
  analogWrite(ENA, abs(rightSpeed));
}

void stopCar() {
  moveCar(0, 0);
}

void rotateDegrees(float degrees) {
  float arcLength = PI * TRACK_WIDTH * (abs(degrees) / 360.0);
  float wheelCircumference = PI * WHEEL_DIAMETER;
  float revolutions = arcLength / wheelCircumference;
  float timeSeconds = (revolutions * 60.0) / MOTOR_RPM;
  int timeMillis = (int)(timeSeconds * 1000 * 1.895);

  // Left wheel backward, right wheel forward for in-place rotation
  if (degrees < 0)
    moveCar(255, -255);
  else
    moveCar(-255, 255);
  delay(timeMillis);
  moveCar(0, 0);  // Stop
}

void rotateDegreesWithUltrasonic(float degrees) {
  float arcLength = PI * TRACK_WIDTH * (abs(degrees) / 360.0);
  float wheelCircumference = PI * WHEEL_DIAMETER;
  float revolutions = arcLength / wheelCircumference;
  float timeSeconds = (revolutions * 60.0) / MOTOR_RPM;
  int timeMillis = (int)(timeSeconds * 1000 * 1.895);

  // Left wheel backward, right wheel forward for in-place rotation
  if (degrees < 0)
    moveCar(255, -255);
  else
    moveCar(-255, 255);
  for (int i = 0; i < timeMillis; i += 5) {
    if (readUltrasonic(ECHO_CENTER) < 20) {
      moveCar(0, 0);  // Stop
      return;
    }
    delay(5);
  }
}
double readUltrasonic(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0)
    return 999;
  return duration * 0.034 / 2.0;
}

void avoidEdge() {
  if (digitalRead(IR_FRONT1) || digitalRead(IR_FRONT2))
    rotateDegrees(180);
  else if (digitalRead(IR_BACK)) {
    moveCar(255, 255);
    DelayWithEdgeAvoidance(50);
  } else
    moveCar(255, 255);
}

void DelayWithEdgeAvoidance(int delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
    avoidEdge();
    delay(5);  // Small delay to prevent blocking
  }
}

void initPins() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_CENTER, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_FRONT2, INPUT);
  pinMode(IR_FRONT1, INPUT);
  pinMode(IR_BACK, INPUT);
}
