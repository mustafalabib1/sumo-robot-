// === Ultrasonic Sensor ===
#define TRIG_PIN 9
#define ECHO_PIN 4


double readUltrasonic();
void initPins();

void setup() {
  Serial.begin(9600);
  initPins();
  randomSeed(analogRead(A5));
}

void loop() {
  Serial.println("Test Ultrasonic");
  double d = readUltrasonic();
  Serial.print("Distance: ");
  Serial.println(d);
  delay(500);
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


void initPins() {

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}
