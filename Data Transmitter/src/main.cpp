/****************************************************************************
 *  Data Transmitter: LiDAR + Encoders → ESP-NOW Transmission to ESP Station
 *       • Reads and packets LiDAR and Encoder data to transmit
 *  Platform:   ESP32-S3 (Arduino-ESP32 core ≥ 3.0.0)
 ****************************************************************************/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "rpLidar.h"
#include "rpLidarTypes.h"
#include "driver/pcnt.h"

/* ----------  USER SETTINGS ----------------------------------- */
static const uint8_t PEER_MAC[6] = { 0x5C,0x01,0x3B,0x96,0x27,0x48 }; // Station ESP32 MAC Addr
#define MSG_TYPE_LIDAR_POINTS  0x10   // followed by N*(angle,dist,qual)
#define MSG_TYPE_ENCODERS      0x11   // followed by 4 × int16_t counts
#define MSG_TYPE_CONTROL       0x12   // Control message
/* -------------------------------------------------------------- */

/* ==========  ENCODERS  ======================================= */
const int ENCODER_A[4]     = {27, 34, 32, 13};
const int ENCODER_B[4]     = {14, 35, 33, 39};
const pcnt_unit_t PCNT_U[4]= {PCNT_UNIT_0,PCNT_UNIT_1,PCNT_UNIT_2,PCNT_UNIT_3};
void   setupEncoder (int idx,int pinA,int pinB,pcnt_unit_t unit);
void   readEncoders(int16_t *dst);

/* ==========  SERIAL  ========================================== */
#define RXD1 26
#define TXD1 25
HardwareSerial motor_serial(1);

/* ==========  LiDAR  ========================================== */
#define RXD2 16
#define TXD2 17
rpLidar *lidar;
void   setupLidar();
int    readLidarOnce();                // returns #valid points

/* ==========  ESP-NOW  ======================================== */
void   setupEspNow();
void   onReceive(const uint8_t*,const uint8_t*,int);
void   onSent   (const uint8_t*, esp_now_send_status_t);

/* Helper: build & send 250-byte LiDAR frames */
class LidarPacketBuilder {
  public:
    void begin() { idx = 0; packet[0] = MSG_TYPE_LIDAR_POINTS; }
    void addPoint(uint8_t angle, uint16_t dist, uint8_t qual) {
      if (idx >= MAX_POINTS) flush();          // frame full
      uint8_t *p = &packet[2 + 4*idx];
      p[0] = angle;               // 1 B
      p[1] = dist & 0xFF;        // LOW byte first 
      p[2] = dist >> 8;          // HIGH byte
      p[3] = qual;                // 1 B
      idx++;
    }
    void flush() {
      if (!idx) return;
      packet[1] = idx;                                    // count
      esp_err_t err = esp_now_send(PEER_MAC, packet, 2 + 4*idx);
      if (err != ESP_OK) Serial.printf("ESP-NOW send error %d\n",err);
      idx = 0;
    }
  private:
    static constexpr uint8_t MAX_POINTS = 62;             // 250-B rule
    uint8_t packet[250];
    uint8_t idx = 0;
} pkt;





/* ====================  SETUP  ================================ */
void setup() {
  Serial.begin(115200);

  motor_serial.begin(115200, SERIAL_8N1 ,RXD1, TXD1);

  /* encoders -------------------------------------------------- */
  for (int i=0;i<4;i++) setupEncoder(i,ENCODER_A[i],ENCODER_B[i],PCNT_U[i]);
  /* LiDAR ----------------------------------------------------- */
  setupLidar();
  /* ESP-NOW --------------------------------------------------- */
  setupEspNow();
  Serial.println("Initialisation complete.");

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

/* ====================  MAIN LOOP  ============================ */
void loop() {
  /* -------- read/send one LiDAR revolution  ----------------- */
  int pts = readLidarOnce();      // builds frames internally
  if (pts==0) delay(20);          // lidar idle → short pause

  /* -------- send encoder snapshot every 25 ms -------------- */
  static uint32_t t0=0;
  if (millis()-t0 > 25) {
    t0 = millis();
    uint8_t packet[1 + 4*sizeof(int16_t)];
    packet[0] = MSG_TYPE_ENCODERS;
    int16_t *pCnt = reinterpret_cast<int16_t*>(&packet[1]);
    readEncoders(pCnt);
    esp_now_send(PEER_MAC, packet, sizeof(packet));
  }
}





/* ==================  IMPLEMENTATIONS  ======================== */
void setupEspNow() {
  WiFi.mode(WIFI_STA);                    // no AP beaconing
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed"); for(;;);
  }
  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0; peer.encrypt = false;
  if (!esp_now_is_peer_exist(PEER_MAC)) esp_now_add_peer(&peer);
}

// Callbacks
void onReceive(const uint8_t*, const uint8_t* data, int) {
  uint8_t type = data[0];
  if (type == MSG_TYPE_CONTROL) {
    char command = data[1];
    motor_serial.print(command); 
  }

}
void onSent(const uint8_t*, esp_now_send_status_t status) {
  //if (status!=ESP_NOW_SEND_SUCCESS) Serial.println("❌ ESP-NOW send failed");
}

/* ----------  LiDAR  ----------------------------------------- */
void setupLidar(){
  Serial2.setRxBufferSize(5000);
  Serial2.begin(460800,SERIAL_8N1,RXD2,TXD2);
  lidar = new rpLidar(&Serial2,460800);
  lidar->resetDevice();
  lidar->setAngleOfInterest(0,360);
  if (lidar->start(standard))
      Serial.println("LiDAR ready");
  else  Serial.println("LiDAR start failed");
}

int readLidarOnce() {
  int n = lidar->readMeasurePoints();
  if (!n) {                          // restart if scan lost
      lidar->resetDevice();
      lidar->start(standard);
      return 0;
  }
  pkt.begin();                       // start new frame-series
  for (int i=0;i<n;i++){
    auto &d = lidar->DataBuffer[i];
    if (d.quality==0 || d.distance_low==0) continue;   // skip bad
    uint8_t  angle = ((d.angle_high<<7) + (d.angle_low>>1))/64;
    uint16_t dist  = (d.distance_high<<8) | d.distance_low;
    uint8_t  qual  = d.quality>>2;
    pkt.addPoint(angle,dist,qual);
  }
  pkt.flush();                       // flush remainder
  return n;
}

/* ----------  Encoders  -------------------------------------- */
void setupEncoder(int idx,int pinA,int pinB,pcnt_unit_t unit){
  pcnt_config_t c{};
  c.pulse_gpio_num  = pinA;
  c.ctrl_gpio_num   = pinB;
  c.channel         = PCNT_CHANNEL_0;
  c.unit            = unit;
  c.pos_mode        = PCNT_COUNT_INC;
  c.neg_mode        = PCNT_COUNT_DEC;
  c.lctrl_mode      = PCNT_MODE_REVERSE;
  c.hctrl_mode      = PCNT_MODE_KEEP;
  c.counter_h_lim   = 32767;
  c.counter_l_lim   = -32768;
  pcnt_unit_config(&c);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void readEncoders(int16_t *dst){
  for(int i=0;i<4;i++) pcnt_get_counter_value(PCNT_U[i], &dst[i]);
}
