// ============ DRONE TX - ESP-NOW ============
// Sticks: throttle G33 (inverted), yaw G32, pitch G35 (inverted), roll G34, arm G16
// Glove (optional): supplies pitch/roll/yaw + throttle-up/land/kill.
//   Falls back to sticks if the glove goes quiet or its IMU goes stale.

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

uint8_t droneMac[] = {0x48,0x9D,0x31,0x04,0xAA,0xF4};

// ---- PINS ----
const int PIN_THR   = 33;
const int PIN_YAW   = 32;
const int PIN_PITCH = 35;
const int PIN_ROLL  = 34;
const int PIN_ARM   = 16;

// ---- stick centres (must match FC) ----
const int THR_CENTER=2128, YAW_CENTER=1909, PITCH_CENTER=2173, ROLL_CENTER=1968;

// ---- glove tuning ----
#define GLOVE_ENABLE     1
#define GLOVE_TIMEOUT_MS 300     // no packet this long = fall back to sticks
#define GLOVE_THR_STEP    18     // ADC counts above centre while index held
#define GLOVE_SPAN      700     // ±1000 glove units -> this many ADC counts

struct __attribute__((packed)) Packet { uint16_t throttle,yaw,pitch,roll; uint8_t arm; };
Packet tx;

// ---- GLOVE PACKET (must match glove sketch byte-for-byte) ----
struct __attribute__((packed)) GlovePacket {
  int16_t pitch, roll, yaw;
  uint8_t active, thrUp, land, kill;
};
GlovePacket glvBuf, glv;
volatile bool glvFresh = false;
unsigned long lastGloveRx = 0;
volatile unsigned long glvCount = 0;

// ---- TELEMETRY (must match FC struct exactly - 41 bytes) ----
struct __attribute__((packed)) Telem {
  float pitch, roll, iR, iP, vib;
  float fVx, fVy;
  float hdg;
  float alt;
  uint16_t thr;
  uint8_t  armd, fQ, fF;
};
Telem telemBuf, telem;
volatile bool telemFresh=false;
unsigned long lastTelemRx=0;

// one callback, two packet types - dispatch on length
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if(len == sizeof(Telem)){
    memcpy((void*)&telemBuf, data, sizeof(Telem));
    telemFresh = true;
  } else if(len == sizeof(GlovePacket)){
    memcpy((void*)&glvBuf, data, sizeof(GlovePacket));
    glvFresh = true;
    glvCount++;                       // <<< NEW
  }
}

unsigned long lastSend=0, lastPrint=0;

void setup(){
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_ARM, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  delay(500);
  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init FAIL");
    while(1){ delay(1000); }
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, droneMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if(esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Add peer FAIL");
    while(1){ delay(1000); }
  }

  Serial.print("TX MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("Telem size: "); Serial.println(sizeof(Telem));
  Serial.print("Glove size: "); Serial.println(sizeof(GlovePacket));
  Serial.println("TX ready");
}

void loop(){
  if(millis()-lastSend < 20){ delay(1); return; }   // 50Hz
  lastSend = millis();

  if(glvFresh){ glvFresh = false; glv = glvBuf; lastGloveRx = millis(); }
  if(telemFresh){ telemFresh = false; telem = telemBuf; lastTelemRx = millis(); }

  // ---- stick baseline ----
  int thrRaw = 4095 - analogRead(PIN_THR);
  tx.yaw   =        analogRead(PIN_YAW);
  tx.pitch = 4095 - analogRead(PIN_PITCH);
  tx.roll  =        analogRead(PIN_ROLL);
  tx.arm   = (digitalRead(PIN_ARM)==LOW) ? 1 : 0;

  // ---- glove override ----
  bool gloveLive = false;
#if GLOVE_ENABLE
  gloveLive = (millis()-lastGloveRx < GLOVE_TIMEOUT_MS) && glv.active;

  if(gloveLive){
    if(glv.kill){
      tx.arm = 0;                                   // kill: disarm immediately
      thrRaw = THR_CENTER;
    } else if(glv.land){
      thrRaw = 0;                                   // below LAND_STICK_THR on the FC
    } else if(glv.thrUp){
      thrRaw = THR_CENTER + GLOVE_THR_STEP + 200;   // clear of THR_DEADBAND
    } else {
      thrRaw = THR_CENTER;                          // freeze
    }

    tx.pitch = constrain(PITCH_CENTER + (glv.pitch * GLOVE_SPAN)/1000, 0, 4095);
    tx.roll  = constrain(ROLL_CENTER  + (glv.roll  * GLOVE_SPAN)/1000, 0, 4095);
    tx.yaw   = constrain(YAW_CENTER   + (glv.yaw   * GLOVE_SPAN)/1000, 0, 4095);
  }
#endif

  tx.throttle = constrain(thrRaw, 0, 4095);
  esp_now_send(droneMac, (uint8_t*)&tx, sizeof(tx));

  if(millis()-lastPrint > 200){
    lastPrint = millis();
    Serial.printf("T:%4u Y:%4u P:%4u R:%4u arm:%u | ",
      tx.throttle, tx.yaw, tx.pitch, tx.roll, tx.arm);
    if(gloveLive){
      Serial.printf("GLV P:%+5d R:%+5d Y:%+5d t:%u l:%u K:%u | ",
        glv.pitch, glv.roll, glv.yaw, glv.thrUp, glv.land, glv.kill);
    } else {
      Serial.printf("GLV --off-- rx:%lu | ", glvCount);
    }
    if(millis()-lastTelemRx < 1000){
      Serial.printf("FC %s thr:%4u P:%5.1f R:%5.1f iR:%5.1f iP:%5.1f vib:%4.2f hdg:%+6.1f alt:%5.2f\n",
        telem.armd?"ARM":"DIS", telem.thr, telem.pitch, telem.roll,
        telem.iR, telem.iP, telem.vib, telem.hdg, telem.alt);
    } else {
      Serial.println("FC --no telem--");
    }
  }
}