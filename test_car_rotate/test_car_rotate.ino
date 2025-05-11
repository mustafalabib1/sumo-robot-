// === Motor Driver Pins (L298N) ===
#define ENA 10
#define IN1 13
#define IN2 12
#define IN3 8
#define IN4 7
#define ENB 5

// === Ultrasonic Sensor ===
#define TRIG_PIN 9
#define ECHO_PIN 4

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

#define TRACK_WIDTH 0.3       // in meters
#define WHEEL_DIAMETER 0.065  // in meters
#define MOTOR_RPM 170         // RPM of the motor

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
  delay(3000);
  rotateDegrees(90);  // Rotate 360 degrees in place
  delay(3000);
  rotateDegrees(-180);  // Rotate 360 degrees in place
  delay(3000);
//   rotateDegrees(45, 255);  
//   delay(3000);
//   rotateDegrees(135, 255); 
//   delay(3000);
//   rotateDegrees(225, 255); 
//   delay(3000);
}


void moveCar(int leftSpeed, int rightSpeed) {
  digitalWrite(IN3, leftSpeed > 0);
  digitalWrite(IN4, leftSpeed < 0);
  analogWrite(ENB, abs(leftSpeed));
  // int FleftSpeed=map(leftSpeed,0,255,0,255*133/170);
  // analogWrite(FENB, abs(FleftSpeed));

  digitalWrite(IN1, rightSpeed > 0);
  digitalWrite(IN2, rightSpeed < 0);
  analogWrite(ENA, abs(rightSpeed));
  // int RleftSpeed=map(rightSpeed,0,255,0,255*133/170);
  // analogWrite(FENA, abs(RleftSpeed));
}
void stopCar() {
  moveCar(0, 0);
}

void rotateDegrees(float degrees) {
  float arcLength = PI * TRACK_WIDTH * (abs(degrees) / 360.0);
  float wheelCircumference = PI * WHEEL_DIAMETER;
  float revolutions = arcLength / wheelCircumference;
  float timeSeconds = (revolutions * 60.0) / MOTOR_RPM;
  int timeMillis = (int)(timeSeconds * 1000*1.895);

  // Left wheel backward, right wheel forward for in-place rotation
  if(degrees<0)
  moveCar(255, -255);
  else 
  moveCar(-255, 255);
  delay(timeMillis);
  moveCar(0, 0);  // Stop
}

void initPins() {
  pinMode(ENA, OUTPUT);
  // pinMode(FENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  // pinMode(FENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_FRONT2, INPUT);
  pinMode(IR_FRONT1, INPUT);
  pinMode(IR_BACK, INPUT);
}
