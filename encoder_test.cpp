// Encoder Pins for 4 Motors
const int ENCODER_A[4] = {34, 36, 32, 25};  // GPIOs for channel A
const int ENCODER_B[4] = {35, 39, 33, 26};  // GPIOs for channel B

volatile long encoderCounts[4] = {0, 0, 0, 0};

void IRAM_ATTR handleEncoder0() {
    bool b = digitalRead(ENCODER_B[0]);
    encoderCounts[0] += b ? 1 : -1;
}
  
void IRAM_ATTR handleEncoder1() {
    bool b = digitalRead(ENCODER_B[1]);
    encoderCounts[1] += b ? 1 : -1;
}
  
void IRAM_ATTR handleEncoder2() {
    bool b = digitalRead(ENCODER_B[2]);
    encoderCounts[2] += b ? 1 : -1;
}
  
void IRAM_ATTR handleEncoder3() {
    bool b = digitalRead(ENCODER_B[3]);
    encoderCounts[3] += b ? 1 : -1;
}

void setup() {
    Serial.begin(115200);
  
    for (int i = 0; i < 4; i++) {
      pinMode(ENCODER_A[i], INPUT);
      pinMode(ENCODER_B[i], INPUT);
    }
  
    // Attach interrupts to Channel A pins
    attachInterrupt(digitalPinToInterrupt(ENCODER_A[0]), handleEncoder0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A[1]), handleEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A[2]), handleEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A[3]), handleEncoder3, CHANGE);
}
  
void loop() {
    static long lastCounts[4] = {0};
  
    for (int i = 0; i < 4; i++) {
      if (encoderCounts[i] != lastCounts[i]) {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(encoderCounts[i]);
        lastCounts[i] = encoderCounts[i];
      }
    }
  
    delay(50);
  }
  