/*****************************************************************
 *  ESP-NOW Station  (RTOS version) → Serial to PC 
 *     • receives LiDAR / encoder frames from rover data transmitter
 *     • dedicated task “SerialTask” drains USB keyboard and forwards
 *       w / a / s / d / x commands (max 3 repeats) via ESP-NOW
 * 
 * Platform:   ESP32-S3 (Arduino-ESP32 core ≥ 3.0.0)
 *****************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

/* ---------- Message-type IDs (must match rover) ------------- */
#define MSG_TYPE_LIDAR_POINTS  0x10
#define MSG_TYPE_ENCODERS      0x11
#define MSG_TYPE_CONTROL       0x12       

/* ---------- Types used by incoming LiDAR packet -------------- */
struct __attribute__((packed)) Point {
  uint8_t  angle;          // 0-255 ⇒ 360 ° * angle / 256
  uint16_t dist_q4;        // 0.25 mm units
  uint8_t  qual;           // quality 0-63
};

/* ---------- Peer (rover) MAC address ------------------------ */
static const uint8_t PEER_MAC[6] = { 0x5C,0x01,0x3B,0x96,0x2E,0x24 };

/* ---------- Forward declarations ---------------------------- */
void        setupEspNow();
void        onReceive(const uint8_t*, const uint8_t*, int);
void        onSent   (const uint8_t*, esp_now_send_status_t);
esp_err_t   sendControl(char cmd);
void        SerialTask(void* arg);     // *** new ***

/* ---------- Command-repeat limiter -------------------------- */
constexpr uint8_t  MAX_REPEATS  = 3;
static    char     last_cmd     = '\0';
static    uint8_t  repeat_count = 0;

/* ---------- ESP-NOW state flag (shared with ISR) ------------ */
volatile bool espnow_busy = false;

/* =====================  SETUP  ============================== */
void setup()
{
  Serial.begin(115200);
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

/* =====================  LOOP  =============================== *
 * Loop is now free; keep it minimal so SerialTask gets plenty
 * of CPU time.  You can do other work here later.              */
void loop()
{
  vTaskDelay(10);   // yield; nothing needed here for now
}

/* ===================  RTOS TASK  ============================ */
void SerialTask(void*)
{
  for (;;)
  {
    /* drain ALL pending bytes each pass */
    while (Serial.available())
    {
      char c = tolower(Serial.read());
      if (!(c=='w'||c=='a'||c=='s'||c=='d'||c=='x'||
            c =='r'||c=='f'||c=='u'||c=='i'||c=='o'||
            c=='j'||c=='k'||c=='l'||c=='n'||c==',')) continue;

      esp_err_t err = sendControl(c);

      /* repeat-limiter */
      // if (c != last_cmd) { last_cmd = c; repeat_count = 0; }

      // if (repeat_count < MAX_REPEATS)
      // {
      // if (sendControl(c) == ESP_OK)
      //   ++repeat_count;           // count only if radio queued it
      //}
    }
    vTaskDelay(1);                  // give up timeslice (≈1 ms)
  }
}

/* --------------- send one control byte via ESP-NOW ---------- */
esp_err_t sendControl(char cmd)
{
  if (espnow_busy) return ESP_ERR_INVALID_STATE;

  uint8_t payload[2] = { MSG_TYPE_CONTROL,
                         static_cast<uint8_t>(cmd) };
  espnow_busy = true;
  return esp_now_send(PEER_MAC, payload, sizeof(payload));
}

/* =================  ESP-NOW plumbing ======================== */
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
      float deg     = pts[i].angle * 360.0f / 256.0f;
      Serial.printf("LIDAR  %.1f°  %.1f mm  q=%u\n",
                    deg, dist_mm, pts[i].qual);
    }
  }
  else if (type == MSG_TYPE_ENCODERS && len == 9) {
    const int16_t* enc = reinterpret_cast<const int16_t*>(&data[1]);
    Serial.printf("ENC   LF=%d  LB=%d  RF=%d  RB=%d\n",
                  enc[0], enc[1], enc[2], enc[3]);
  }
}
