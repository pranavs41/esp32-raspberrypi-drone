// ============ DRONE TX - ESP-NOW ============
// Left stick:  vertical=throttle (G33, inverted), horizontal=yaw (G32)
// Right stick: vertical=pitch    (G35, inverted), horizontal=roll (G34)
//
// 2026-08-02: Telem struct extended with flow fields (fVx,fVy,fQ,fF)
//             MUST match FC struct byte-for-byte or telemetry goes silent.

#include <esp_now.h>
#include <WiFi.h>

uint8_t droneMac[] = {0xB4,0xBF,0xE9,0x60,0x89,0xD0};  // FC MAC

// ---- PINS ----
const int PIN_THR   = 33;   // left stick vertical (inverted)
const int PIN_YAW   = 32;   // left stick horizontal
const int PIN_PITCH = 35;   // right stick vertical (inverted)
const int PIN_ROLL  = 34;   // right stick horizontal
const int PIN_ARM   = 16;

struct __attribute__((packed)) Packet { uint16_t throttle,yaw,pitch,roll; uint8_t arm; };
Packet tx;

// ---- TELEMETRY (must match FC struct exactly - 33 bytes) ----
struct __attribute__((packed)) Telem {
  float pitch, roll, iR, iP, vib;
  float fVx, fVy;
  float hdg;                    // <<< NEW
  uint16_t thr;
  uint8_t  armd, fQ, fF;
};
Telem telemBuf;                 // written by callback
Telem telem;                    // stable copy for printing
volatile bool telemFresh=false;
unsigned long lastTelemRx=0;

// keep this SHORT - runs in WiFi task context, no Serial in here
void onTelem(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if(len == sizeof(Telem)){
    memcpy((void*)&telemBuf, data, sizeof(Telem));
    telemFresh = true;
  }
}

unsigned long lastSend=0, lastPrint=0;

void setup(){
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_ARM, INPUT_PULLUP);   // switch to GND = ARMED

  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init FAIL");
    while(1){ delay(1000); }
  }

  esp_now_register_recv_cb(onTelem);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, droneMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if(esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Add peer FAIL");
    while(1){ delay(1000); }
  }
  Serial.print("TX MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("Telem struct size: "); Serial.println(sizeof(Telem));  // <<< NEW: must print 33
  Serial.println("TX ready");
}

void loop(){
  if(millis()-lastSend < 20){ delay(1); return; }   // <<< NEW: delay(1) yields to idle task (WDT)
  lastSend = millis();

  tx.throttle = 4095 - analogRead(PIN_THR);    // invert: stick up = high
  tx.yaw      =        analogRead(PIN_YAW);
  tx.pitch    = 4095 - analogRead(PIN_PITCH);  // invert: stick up = high
  tx.roll     =        analogRead(PIN_ROLL);
  tx.arm      = (digitalRead(PIN_ARM)==LOW) ? 1 : 0;

  esp_now_send(droneMac, (uint8_t*)&tx, sizeof(tx));

  if(telemFresh){
    telemFresh = false;
    telem = telemBuf;
    lastTelemRx = millis();
  }

  if(millis()-lastPrint > 200){
    lastPrint = millis();
    Serial.printf("T:%4u Y:%4u P:%4u R:%4u arm:%u | ",
      tx.throttle, tx.yaw, tx.pitch, tx.roll, tx.arm);
    if(millis()-lastTelemRx < 1000){
      Serial.printf("FC %s thr:%4u P:%5.1f R:%5.1f iR:%5.1f iP:%5.1f vib:%4.2f",
        telem.armd?"ARM":"DIS", telem.thr, telem.pitch,
        telem.roll, telem.iR, telem.iP, telem.vib);
      Serial.printf(" | fVx:%+6.2f fVy:%+6.2f fQ:%3u fF:%u hdg:%+6.1f\n",
        telem.fVx, telem.fVy, telem.fQ, telem.fF, telem.hdg);
    } else {
      Serial.println("FC --no telem--");
    }
  }
}