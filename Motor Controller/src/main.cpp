/*****************************************************************
 *  Motor Controller
 *     • Receives serial commands from Data Transmitter
 *     • Controls Motors for direction and speed
 * 
 * Platform:   ESP32-S3 (Arduino-ESP32 core ≥ 3.0.0)
 *****************************************************************/

#include <Arduino.h>

// --- PIN DEFINITIONS AND OBJECTS ---

//           Motors:      M1  M2  M3  M4
//           Position:    LF  LB  RF  RB
const int MOTOR_ENA[4] = {27, 14, 23, 22};      // PWM pins for Motor 1–4

const int MOTOR_IN1[4] = {26, 21,  4, 18};     // IN1/IN3 for Motor 1–4
const int MOTOR_IN2[4] = {13, 16,  17, 19};     // IN2/IN4 for Motor 1–4
//                     = {IN1,IN3,IN1,IN3}
//                     = {IN2,IN4,IN2,IN4}

#define LF 1
#define LB 2
#define RF 3
#define RB 4

// --- PWM SETUP ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES  = 8;      // 8-bit resolution
const int MOTOR_PWM_CH[4] = {0, 1, 2, 3};  // Hardware PWM channels for Motor 1–4

// --- STATE COMMANDS ---
#define STOP          'x'
#define FORWARD       'w'
#define BACKWARD      's'
#define LEFT          'a'
#define RIGHT         'd'

#define STRAFE_LEFT   'j'
#define STRAFE_RIGHT  'l'
#define U_DIAG_LEFT   'u'
#define U_DIAG_RIGHT  'o'
#define D_DIAG_LEFT   'n'
#define D_DIAG_RIGHT  ','

#define ALT_FORWARD   'i'
#define ALT_BACKWARD  'k'

#define SPEED_INC   'r'
#define SPEED_DEC   'f'


/* ==========  SERIAL  ========================================== */
#define RXD1 33
#define TXD1 32
HardwareSerial collector_serial(1);


// === FUNCTION DECLARATIONS === 
void setMotorSpeed(int motorID, int speed);
void brakeMotor(int motorID);
void motorTest();


// === VARIABLES === 
char cmd      = STOP;
char prev_cmd = '.';
int speed     = 30;


// SETUP
void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  collector_serial.begin(115200, SERIAL_8N1, RXD1, TXD1); // UART to collector ESP32

  // Initialize motor pins
  for (int i = 0; i < 4; i++) {

    //Serial.println("Setting up motor");

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

// MAIN LOOP
void loop() {

  if (collector_serial.available()) {
    prev_cmd = cmd;
    cmd = collector_serial.read();
    cmd = tolower(cmd);
    // Serial.println(cmd);
  }

  switch(cmd){
    case STOP:
      brakeMotor(LF);
      brakeMotor(RF);
      brakeMotor(LB);
      brakeMotor(RB);
      break;
      
    case FORWARD:
      setMotorSpeed(LF, speed);
      setMotorSpeed(RF, speed);
      setMotorSpeed(LB, speed);
      setMotorSpeed(RB, speed);
      break;
    
    case BACKWARD:
      setMotorSpeed(LF, -speed);
      setMotorSpeed(RF, -speed);
      setMotorSpeed(LB, -speed);
      setMotorSpeed(RB, -speed);
      break;

    case LEFT:
      setMotorSpeed(LF, -speed);
      setMotorSpeed(RF, speed);
      setMotorSpeed(LB, -speed);
      setMotorSpeed(RB, speed);
      break;

    case RIGHT:
      setMotorSpeed(LF, speed);
      setMotorSpeed(RF, -speed);
      setMotorSpeed(LB, speed);
      setMotorSpeed(RB, -speed);
      break;
    
    case ALT_FORWARD:
      setMotorSpeed(LF, speed);
      setMotorSpeed(RF, speed);
      setMotorSpeed(LB, speed);
      setMotorSpeed(RB, speed);
      break;

    case ALT_BACKWARD:
      setMotorSpeed(LF, -speed);
      setMotorSpeed(RF, -speed);
      setMotorSpeed(LB, -speed);
      setMotorSpeed(RB, -speed);
      break;

    case STRAFE_LEFT:
      setMotorSpeed(LF, -speed);
      setMotorSpeed(RF, speed);
      setMotorSpeed(LB, speed);
      setMotorSpeed(RB, -speed);
      break;


    case STRAFE_RIGHT:
      setMotorSpeed(LF, speed);
      setMotorSpeed(RF, -speed);
      setMotorSpeed(LB, -speed);
      setMotorSpeed(RB, speed);
      break;

    case U_DIAG_LEFT:
      brakeMotor(LF);
      setMotorSpeed(RF, speed);
      setMotorSpeed(LB, speed);
      brakeMotor(RB);
      break;

    case U_DIAG_RIGHT:
      setMotorSpeed(LF, speed);
      brakeMotor(RF);
      brakeMotor(LB);
      setMotorSpeed(RB, speed);
      break;

    case D_DIAG_LEFT:
      setMotorSpeed(LF, -speed);
      brakeMotor(RF);
      brakeMotor(LB);
      setMotorSpeed(RB, -speed);
      break;

    case D_DIAG_RIGHT:
      brakeMotor(LF);
      setMotorSpeed(RF, -speed);
      setMotorSpeed(LB, -speed);
      brakeMotor(RB);
      break;

    case SPEED_INC:
      speed += 1;
      cmd = prev_cmd;
      break;

    case SPEED_DEC:
      speed -= 1;
      cmd = prev_cmd;
      break;
    
  }

 
}



// ===== FUNCTION DEFINITIONS ===== 

// === Set Motor Speed ===
// motorID: 1 to 4
// speed: -100 to 100
void setMotorSpeed(int motorID, int speed) {
  if (motorID < 1 || motorID > 4) return;

  int index = motorID - 1;

  speed = map(speed, -100, 100, -255, 255);
  speed = constrain(speed, -177, 177);

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