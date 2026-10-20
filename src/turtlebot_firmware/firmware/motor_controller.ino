const uint8_t PIN_PWM = 9;
const uint8_t PIN_DIR_A = 12;
const uint8_t PIN_DIR_B = 13;

float targetSpeed = 0.0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);

  digitalWrite(PIN_DIR_A, HIGH);
  digitalWrite(PIN_DIR_B, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    targetSpeed = Serial.parseFloat();
  }
  
  int pwmSignal = targetSpeed * 100;
  analogWrite(PIN_PWM, pwmSignal);
}
