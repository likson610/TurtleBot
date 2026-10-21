const uint8_t PIN_PWM_R = 9;
const uint8_t PIN_PWM_L = 11;
const uint8_t PIN_DIR_L2 = 8;
const uint8_t PIN_DIR_L1 = 7;
const uint8_t PIN_DIR_R2 = 13;
const uint8_t PIN_DIR_R1 = 12;

const uint8_t ENC_R_A = 3;
const uint8_t ENC_R_B = 5;
const uint8_t ENC_L_A = 2;
const uint8_t ENC_L_B = 4;

volatile unsigned int countR = 0;
volatile unsigned int countL = 0;
volatile char signR = 'p';
volatile char signL = 'p';

unsigned long prevTime = 0;
const unsigned long dt = 100;

double measuredR = 0.0;
double measuredL = 0.0;

void setMotorDir(uint8_t pin1, uint8_t pin2, bool state) {
  digitalWrite(pin1, state ? HIGH : LOW);
  digitalWrite(pin2, state ? LOW : HIGH);
}

void setup() {
  pinMode(ENC_R_B, INPUT);
  pinMode(ENC_L_B, INPUT);
  
  setMotorDir(PIN_DIR_R1, PIN_DIR_R2, true);
  setMotorDir(PIN_DIR_L1, PIN_DIR_L2, true);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, RISING);
}

void loop() {
  unsigned long now = millis();
  if (now - prevTime >= dt) {
    measuredR = (countR * 600.0 / 385.0) * 0.10472;
    measuredL = (countL * 600.0 / 385.0) * 0.10472;
    
    Serial.print("r"); Serial.print(signR); Serial.print(measuredR);
    Serial.print(",l"); Serial.print(signL); Serial.println(measuredL);
    
    prevTime = now;
    countR = 0;
    countL = 0;

    analogWrite(PIN_PWM_R, 100);
    analogWrite(PIN_PWM_L, 100);
  }
}

void isrRight() {
  signR = digitalRead(ENC_R_B) ? 'p' : 'n';
  countR++;
}

void isrLeft() {
  signL = digitalRead(ENC_L_B) ? 'n' : 'p';
  countL++;
}
