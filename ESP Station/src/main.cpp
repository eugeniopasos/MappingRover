/*****************************************************************
 *  ESP-NOW Station  (RTOS version) → Serial to PC 
 * 
 *     • Receives LiDAR / encoder frames from rover data transmitter
 *     • Dedicated task “SerialTask” drains incomming key commands 
 *       and target control angles
 * 
 * Platform:   ESP32-S3 (Arduino-ESP32 core ≥ 3.0.0)
 *****************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

/* ---------- Message-type IDs (must match rover) ------------- */
#define MSG_TYPE_LIDAR_POINTS  0x10
#define MSG_TYPE_ENCODERS      0x11
#define MSG_TYPE_COMMAND       0x12
#define MSG_TYPE_CONTROL       0x13

/* ---------- Types used by incoming LiDAR packet -------------- */
struct __attribute__((packed)) Point {
  float  angle;          
  uint16_t dist_q4;        // 0.25 mm units
  uint8_t  qual;           // quality 0-63
};

/* ---------- Encoder Packet -------------- */
#pragma pack(push,1)                 // no padding bytes
struct EncoderYawPacket {
    uint8_t  type;        // MSG_TYPE_ENCODERS
    int16_t  counts[4];   // wheel encoder values
    float    yaw;         // most‑recent yaw from serial
};
#pragma pack(pop)


/* ---------- Peer (rover) MAC address ------------------------ */
static const uint8_t PEER_MAC[6] = { 0x5C,0x01,0x3B,0x96,0x2E,0x24 };

/* ---------- Forward declarations ---------------------------- */
void        setupEspNow();
void        onReceive(const uint8_t*, const uint8_t*, int);
void        onSent   (const uint8_t*, esp_now_send_status_t);
esp_err_t sendData(uint8_t msgType, const void* data, size_t len);
void        SerialTask(void* arg);     // *** new ***

/* ---------- ESP-NOW state flag (shared with ISR) ------------ */
volatile bool espnow_busy = false;

/* =====================  SETUP  ============================== */
void setup()
{
  Serial.begin(921600);
  Serial.setRxBufferSize(1024);        // room for key-mash bursts
  setupEspNow();

  /* ---------- spawn keyboard-drain task on core 1 ------------ */
  xTaskCreatePinnedToCore(
      SerialTask,              // task entry
      "SerialTask",            // debug name
      2048,                    // stack words ⇒ 8 kB
      nullptr,                 // no parameter
      2,                       // priority (higher than loopTask=1)
      nullptr,                 // don’t need a handle back
      1                        // pin to APP_CPU (core-1, same as loop())
  );
}

/* =====================  LOOP  =============================== */
void loop()
{
  vTaskDelay(10);   // yield; nothing needed here for now
}

/* ===================  RTOS TASK  ============================ */
void SerialTask(void*) {
  for (;;) {
    // do we have at least 1 byte (the type)?
    if (Serial.available() < 1) {
      vTaskDelay(1);
      continue;
    }

    uint8_t msgType = Serial.read();  // consume the type byte

    if (msgType == MSG_TYPE_COMMAND) {
      // wait until the single command byte arrives
      while (Serial.available() < 1) { vTaskDelay(1); }

      char cmd = Serial.read();        // command character
      sendData(MSG_TYPE_COMMAND, &cmd, 1);
    } 

    else if (msgType == MSG_TYPE_CONTROL) {
      // wait for the full 4-byte float
      while (Serial.available() < sizeof(float)) { vTaskDelay(1); }

      float val;
      Serial.readBytes((char*)&val, sizeof(val));
      sendData(MSG_TYPE_CONTROL, &val, sizeof(val));
      //Serial.println(val);

    } 
    else {continue;}
  }
}

/* --------------- send one control byte via ESP-NOW ---------- */
esp_err_t sendData(uint8_t msgType, const void* data, size_t len) {

  if (espnow_busy) return ESP_ERR_INVALID_STATE;

  uint8_t payload[1 + len];
  payload[0] = msgType;
  memcpy(payload + 1, data, len);

  espnow_busy = true;

  return esp_now_send(PEER_MAC, payload, sizeof(payload));
}

/* =================  ESP-NOW ======================== */
void setupEspNow()
{
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");  while (true) {}
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(PEER_MAC))
      esp_now_add_peer(&peer);

  Serial.println(F("Bridge ready – LiDAR/encoders flowing; type WASDX…"));
}

/* --------------- TX callback: free the token ---------------- */
void onSent(const uint8_t*, esp_now_send_status_t)
{
  espnow_busy = false;               // allow next packet
}

/* --------------- RX callback: debug printouts --------------- */
void onReceive(const uint8_t*, const uint8_t* data, int len)
{
  uint8_t type = data[0];

  if (type == MSG_TYPE_LIDAR_POINTS && len >= 2) {
    uint8_t count = data[1];
    if (len != 2 + count * sizeof(Point)) {
      Serial.println("⚠️  LiDAR packet length mismatch"); return;
    }
    const Point* pts = reinterpret_cast<const Point*>(&data[2]);
    for (uint8_t i = 0; i < count; ++i) {
      float dist_mm = pts[i].dist_q4 * 0.25f;
      float deg     = pts[i].angle; //* 360.0f / 256.0f;
      Serial.printf("L,%.1f,%.1f,%u\n", deg, dist_mm, pts[i].qual);
      // Serial.printf("LIDAR  %.1f°  %.1f mm  q=%u\n",
      //               deg, dist_mm, pts[i].qual);
    }
  }
  else if (type == MSG_TYPE_ENCODERS && len == sizeof(EncoderYawPacket)) {

    auto *p = reinterpret_cast<const EncoderYawPacket *>(data);

    Serial.printf("E,%d,%d,%d,%d,%.2f\n",
                  p->counts[0],
                  p->counts[1],
                  -p->counts[2],
                  -p->counts[3],
                  p->yaw);   
  }
}
