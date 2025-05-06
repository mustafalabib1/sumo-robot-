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
  rotateDegrees(90, 150);  // Rotate 90 degrees in place
  delay(1000);
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
