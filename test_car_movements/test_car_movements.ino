#include <Arduino.h>
// === Motor Driver Pins (L298N) ===
#define ENA 11
#define FENA 10
#define IN1 13
#define IN2 12
#define IN3 8
#define IN4 7
#define ENB 6
#define FENB 5


void moveCar(int leftSpeed, int rightSpeed);
void stopCar();


void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(FENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(FENB, OUTPUT);
}

void loop() {
  // Move forward at full speed for 2 seconds
  moveCar(128, 128);
  delay(2000);
  // Move backward at half speed for 2 seconds
  moveCar(-128, -128);
  delay(2000);
  // Turn left at full speed for 1 second
  moveCar(255, -255);
  delay(1000);
  // Turn right at full speed for 1 second
  stopCar();
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
