const uint8_t PIN_PWM_R = 9;
const uint8_t PIN_PWM_L = 11;
const uint8_t PIN_DIR_L2 = 8;
const uint8_t PIN_DIR_L1 = 7;
const uint8_t PIN_DIR_R2 = 13;
const uint8_t PIN_DIR_R1 = 12;

void setMotorDir(uint8_t pin1, uint8_t pin2, bool state) {
  digitalWrite(pin1, state ? HIGH : LOW);
  digitalWrite(pin2, state ? LOW : HIGH);
}

void setup() {
  pinMode(PIN_PWM_R, OUTPUT);
  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_DIR_R1, OUTPUT);
  pinMode(PIN_DIR_R2, OUTPUT);
  pinMode(PIN_DIR_L1, OUTPUT);
  pinMode(PIN_DIR_L2, OUTPUT);

  setMotorDir(PIN_DIR_R1, PIN_DIR_R2, true);
  setMotorDir(PIN_DIR_L1, PIN_DIR_L2, true);

  Serial.begin(115200);
  Serial.println("Podaj moc (0-255):");
}

void loop() {
  if (Serial.available() > 0) {
    int moc = Serial.parseInt();
    if (moc >= 0 && moc <= 255) {
      analogWrite(PIN_PWM_R, moc);
      analogWrite(PIN_PWM_L, moc);
      Serial.print("Ustawiono moc na: ");
      Serial.println(moc);
    }
  }
}
