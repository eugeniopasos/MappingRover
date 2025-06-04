/*****************************************************************
 *  Motor Controller
 *     • Receives serial commands from Data Transmitter
 *     • Controls Motors for direction and speed
 * 
 * Platform:   ESP32-S3 (Arduino-ESP32 core ≥ 3.0.0)
 * 
 * Reference: SparkFun_ICM-20948_ArduinoLibrary
 * https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary.git
 *****************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"

/* ==========  IMU  ========================================== */

ICM_20948_I2C imu;    // I2C specialization
#define SDA 25
#define SCL 33

/* ==========  MOTOR  ========================================== */

// --- Pin Definitions and Objects ---

//           Motors:      M1  M2  M3  M4
//           Position:    LF  LB  RF  RB
const int MOTOR_ENA[4] = {27, 14, 23, 22};      // PWM pins for Motor 1–4

const int MOTOR_IN1[4] = {26, 21,  4, 18};     // IN1/IN3 for Motor 1–4
const int MOTOR_IN2[4] = {13, 16,  17, 19};     // IN2/IN4 for Motor 1–4
//                     = {IN1,IN3,IN1,IN3}
//                     = {IN2,IN4,IN2,IN4}


// --- Motor IDs ---
#define LF 1
#define LB 2
#define RF 3
#define RB 4

// --- PWM Setup ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES  = 8;      // 8-bit resolution
const int MOTOR_PWM_CH[4] = {0, 1, 2, 3};  // Hardware PWM channels for Motor 1–4

// --- State Commands ---
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

#define SPEED_INC     'r'
#define SPEED_DEC     'f'

#define MODE_CHANGE   't'

#define MSG_TYPE_COMMAND       0x12   // Command message
#define MSG_TYPE_CONTROL       0x13


/* ==========  SERIAL  ========================================== */

// --- ESP32 Serial Pins ---
#define RXD1 35
#define TXD1 32

HardwareSerial collector_serial(1);


// === FUNCTION DECLARATIONS === 
void setMotorSpeed(int motorID, int speed);
void brakeMotor(int motorID);
void motorTest();
void setupIMU();
bool readIMU();
int pid(float current_yaw, float goal_yaw);


// === VARIABLES === 
char cmd      = STOP;
char prev_cmd = '.';
int speed     = 45;

float yaw, prev_yaw, target_yaw;

int correction = 0;

bool autonomous = false; // mode

// SETUP
void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  collector_serial.begin(460800, SERIAL_8N1, RXD1, TXD1); // UART to collector ESP32

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
  Serial.println("Motors Initialized!");

  setupIMU();
  Serial.println("IMU Initialized!");

  Serial.print("Calibrating IMU"); // Waits for IMU to settle
  uint8_t i = 0;
  while (i < 9) {
    Serial.print('.');
    delay(1000);
    i++;
  }

  readIMU();
  Serial.print(" IMU Calibrated!\n");
  digitalWrite(2, HIGH); // Onboard LED notifies IMU is initalized
}

// MAIN LOOP
void loop() {

  if (readIMU()) {
    collector_serial.println(yaw, 2);
  }

  if (collector_serial.available()) {
    char msg_type = collector_serial.read();

    if (msg_type == MSG_TYPE_COMMAND) {
      prev_cmd = cmd;
      cmd = collector_serial.read();
      cmd = tolower(cmd);

      if (cmd == MODE_CHANGE) {   // change mode
        autonomous = !autonomous;
        prev_cmd = STOP;
        cmd = STOP;
      }
    }

    else if (msg_type == MSG_TYPE_CONTROL) {
      cmd = FORWARD;
      float robot_goal_yaw;
      collector_serial.readBytes(
        reinterpret_cast<char*>(&robot_goal_yaw),
        sizeof(robot_goal_yaw)
      );
      target_yaw = -robot_goal_yaw + yaw; // convert robot ref to global ref
 
    }
  }

  // Manuel Mode
  if (!autonomous) {
    switch(cmd){
      case STOP:
        brakeMotor(LF);
        brakeMotor(RF);
        brakeMotor(LB);
        brakeMotor(RB);
        break;
        
      case FORWARD:
        if (prev_cmd != FORWARD) {
          prev_yaw = yaw;
          prev_cmd = FORWARD;
        }
        correction = pid(yaw, prev_yaw);
        setMotorSpeed(LF, speed - correction);
        setMotorSpeed(RF, speed + correction);
        setMotorSpeed(LB, speed - correction);
        setMotorSpeed(RB, speed + correction);

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
  // Autonomous Mode
  else {

    if (cmd == STOP) {
      brakeMotor(LF);
      brakeMotor(RF);
      brakeMotor(LB);
      brakeMotor(RB); 
    }

    else {
      correction = pid(yaw, target_yaw);

      setMotorSpeed(LF, speed - correction);
      setMotorSpeed(RF, speed + correction);
      setMotorSpeed(LB, speed - correction);
      setMotorSpeed(RB, speed + correction);

    }



  }
 
} // loop end



// ===== FUNCTION DEFINITIONS ===== 

void setupIMU() {

  Serial.println("Setting up IMU");

  // 1) Start I2C on the right pins
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);

  // 2) Probe the chip
  if (imu.begin(Wire) != ICM_20948_Stat_Ok) {
    Serial.println("❌ IMU init failed!");
    while (1) { delay(500); } 
  }
  Serial.println("✅ IMU found");

  // 3) Configure DMP
  if (imu.initializeDMP() != ICM_20948_Stat_Ok) {
    Serial.println("❌ DMP init failed!");
    while (1) { delay(500); }
  }

  // 4) Enable the DMP’s quaternion/orientation output

  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) {
    Serial.println("❌ DMP sensor failed!");
    while (1) { delay(500); }
  }
  
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("❌ FIFO enable failed!");
    while (1) { delay(500); }
  }

  if (imu.enableDMP() != ICM_20948_Stat_Ok) {
    Serial.println("❌ DMP enable failed!");
    while (1) { delay(500); }
  }

  if (imu.resetDMP() != ICM_20948_Stat_Ok) {
    Serial.println("❌ DMP reset failed!");
    while (1) { delay(500); }
  }

  if (imu.resetFIFO() != ICM_20948_Stat_Ok) {
    Serial.println("❌ FIFO reset failed!");
    while (1) { delay(500); }
  }
 
}

bool readIMU() {
  static bool initialized = false;           // Run-once flag
  static float offset_yaw   = 0.0;

  icm_20948_DMP_data_t data;
  if (imu.readDMPdataFromFIFO(&data) != ICM_20948_Stat_Ok) {
    return false;
  }

  // Convert quaternion
  double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
  double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
  double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

  double q0_sq = 1.0 - (q1*q1 + q2*q2 + q3*q3);
  if (q0_sq < 0.0) q0_sq = 0.0;          // avoid negative due to tiny rounding errors
  double q0 = sqrt(q0_sq);  

  double qw = q0;
  double qx = q2;
  double qy = q1;
  double qz = -q3;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (qw * qz + qx * qy);
  double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = atan2(t3, t4) * 180.0 / PI;

  // === Set initial reference ===
  if (!initialized) {
    offset_yaw   = yaw;
    initialized = true;
  }

  // === Apply offset ===
  yaw   -= offset_yaw;

  return true;
}

// === Set Motor Speed ===
// motorID: 1 to 4
// speed: -100 to 100
void setMotorSpeed(int motorID, int speed) {
  if (motorID < 1 || motorID > 4) return;

  int index = motorID - 1;

  if (index < 2) {
    speed = speed * 1.065;
  }

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

int pid(float current_yaw, float goal_yaw) {

  uint8_t p = 1;

  float error = current_yaw - goal_yaw;
  // Transition fix
  if (abs(error) >= 180) {
    if (error >= 0) {
      error -= 360;
    }
    else {
      error += 360;
    }
  }

  int correction = round(p * error);  

  return correction;
}