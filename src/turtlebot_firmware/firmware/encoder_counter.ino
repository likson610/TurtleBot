const uint8_t PIN_PWM = 9;
const uint8_t PIN_DIR_A = 12;
const uint8_t PIN_DIR_B = 13;

const uint8_t ENC_CH_A = 3;
const uint8_t ENC_CH_B = 5;

volatile unsigned int pulseCount = 0;
volatile char rotationDir = 'p';

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(ENC_CH_B, INPUT);

  digitalWrite(PIN_DIR_A, HIGH);
  digitalWrite(PIN_DIR_B, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_CH_A), handleEncoder, RISING);
}

void loop() {
  double velocityRad = (pulseCount * 600.0 / 385.0) * 0.10472;
  
  String serialMsg = "r";
  serialMsg += rotationDir;
  serialMsg += velocityRad;
  
  Serial.println(serialMsg);
  
  pulseCount = 0;
  analogWrite(PIN_PWM, 100);
  delay(100);
}

void handleEncoder() {
  rotationDir = digitalRead(ENC_CH_B) ? 'p' : 'n';
  pulseCount++;
}
