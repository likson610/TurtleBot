#include <PID_v1.h>

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

char activeMotor = '0';
bool stateR = true;
bool stateL = true;
String buf = "";

double setR = 0.0;
double setL = 0.0;
double measuredR = 0.0;
double measuredL = 0.0;
double outR = 0.0;
double outL = 0.0;

PID pidR(&measuredR, &outR, &setR, 11.5, 7.5, 0.1, DIRECT);
PID pidL(&measuredL, &outL, &setL, 12.8, 8.3, 0.1, DIRECT);

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
  
  pinMode(ENC_R_B, INPUT);
  pinMode(ENC_L_B, INPUT);

  setMotorDir(PIN_DIR_R1, PIN_DIR_R2, true);
  setMotorDir(PIN_DIR_L1, PIN_DIR_L2, true);

  pidR.SetMode(AUTOMATIC);
  pidL.SetMode(AUTOMATIC);
  
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, RISING);
}

void loop() {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == 'r') {
      activeMotor = 'r';
      buf = "";
    } 
    else if (inChar == 'l') {
      activeMotor = 'l';
      buf = "";
    } 
    else if (inChar == 'p' || inChar == 'n') {
      bool fwd = (inChar == 'p');
      if (activeMotor == 'r' && stateR != fwd) {
        stateR = fwd;
        setMotorDir(PIN_DIR_R1, PIN_DIR_R2, stateR);
      } 
      else if (activeMotor == 'l' && stateL != fwd) {
        stateL = fwd;
        setMotorDir(PIN_DIR_L1, PIN_DIR_L2, stateL);
      }
    } 
    else if (inChar == ',') {
      if (activeMotor == 'r') {
        setR = buf.toFloat();
      } 
      else if (activeMotor == 'l') {
        setL = buf.toFloat();
      }
      buf = "";
    } 
    else if (isDigit(inChar) || inChar == '.') {
      buf += inChar;
    }
  }

  unsigned long now = millis();
  if (now - prevTime >= dt) {
    measuredR = (countR * 600.0 / 385.0) * 0.10472;
    measuredL = (countL * 600.0 / 385.0) * 0.10472;
    
    pidR.Compute();
    pidL.Compute();

    if (setR == 0.0) outR = 0.0;
    if (setL == 0.0) outL = 0.0;

    Serial.print("r");
    Serial.print(signR);
    Serial.print(measuredR);
    Serial.print(",l");
    Serial.print(signL);
    Serial.print(measuredL);
    Serial.println(",");
    
    prevTime = now;
    countR = 0;
    countL = 0;

    analogWrite(PIN_PWM_R, outR);
    analogWrite(PIN_PWM_L, outL);
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