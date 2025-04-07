// === Pin Definitions ===
const int MOTOR_ENA[4] = {13, 21, 4, 27};      // PWM pins for Motor 1–4
const int MOTOR_IN1[4] = {23, 19, 16, 14};     // IN1/IN3 for Motor 1–4
const int MOTOR_IN2[4] = {22, 18, 17,  5};     // IN2/IN4 for Motor 1–4

// === PWM Setup ===
const int PWM_FREQ = 1000;   // 1 kHz
const int PWM_RES  = 8;      // 8-bit resolution
const int MOTOR_PWM_CH[4] = {0, 1, 2, 3};  // Hardware PWM channels for Motor 1–4

void setup() {
  // Initialize motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(MOTOR_ENA[i], OUTPUT);

    // Attach PWM to ENA pins
    ledcSetup(MOTOR_PWM_CH[i], PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_ENA[i], MOTOR_PWM_CH[i]);

    // Brake all motors initially
    brakeMotor(i + 1);
  }
}

// === Set Motor Speed ===
// motorID: 1 to 4
// speed: -255 to 255
void setMotorSpeed(int motorID, int speed) {
  if (motorID < 1 || motorID > 4) return;

  int index = motorID - 1;
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);

  if (speed > 0) {
    digitalWrite(MOTOR_IN1[index], HIGH);
    digitalWrite(MOTOR_IN2[index], LOW);
  } else if (speed < 0) {
    digitalWrite(MOTOR_IN1[index], LOW);
    digitalWrite(MOTOR_IN2[index], HIGH);
  } else {
    brakeMotor(motorID);
    return;
  }

  ledcWrite(MOTOR_PWM_CH[index], pwm);
}

// === Brake a Specific Motor ===
void brakeMotor(int motorID) {
  if (motorID < 1 || motorID > 4) return;

  int index = motorID - 1;
  digitalWrite(MOTOR_IN1[index], LOW);
  digitalWrite(MOTOR_IN2[index], LOW);
  ledcWrite(MOTOR_PWM_CH[index], 0);
}

// === Example Test ===
void loop() {
  // Sweep Motor 1 and Motor 3 forward
  for (int i = 0; i <= 255; i += 5) {
    setMotorSpeed(1, i);
    setMotorSpeed(3, i);
    delay(10);
  }

  // Sweep reverse
  for (int i = 255; i >= -255; i -= 5) {
    setMotorSpeed(1, i);
    setMotorSpeed(3, i);
    delay(10);
  }

  // Brake
  brakeMotor(1);
  brakeMotor(3);
  delay(500);
}
