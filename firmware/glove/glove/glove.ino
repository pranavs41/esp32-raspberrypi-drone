// ============ DRONE GLOVE - ESP-NOW SENDER ============
// Tilt fwd/back = pitch  |  tilt left/right = roll
// RING held: left/right tilt becomes YAW instead of roll (right tilt = CW)
// INDEX held = throttle up  |  MIDDLE held = land
// PINKY = KILL  |  FIST (both halls) = secondary KILL
// ARM stays on the TX physical switch - deliberately not on the glove.

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

uint8_t txMac[] = {0x70,0x4B,0xCA,0x45,0xED,0x5C};   // TX MAC

Adafruit_BNO08x bno08x;
sh2_SensorValue_t imuValue;

// ---- PINS ----
const int PINCH_INDEX  = 13;   // throttle up
const int PINCH_MIDDLE = 14;   // land
const int PINCH_RING   = 27;   // yaw modifier
const int PINCH_PINKY  = 26;   // KILL
const int HALL_INDEX   = 32;
const int HALL_RING    = 25;   // both halls = fist = secondary KILL

// ---- TILT CALIBRATION ----
const float ROLL_NEUTRAL  =   5.6f;
const float PITCH_NEUTRAL = -21.2f;
const float TILT_DEADZONE =   8.0f;   // deg of hand tilt that does nothing
const float TILT_FULL     =  40.0f;   // deg for full deflection

// ---- PACKET (must match TX's GlovePacket byte-for-byte) ----
struct __attribute__((packed)) GlovePacket {
  int16_t pitch, roll, yaw;   // -1000..+1000, 0 = centred
  uint8_t active;             // 1 = IMU data valid
  uint8_t thrUp;              // index held
  uint8_t land;               // middle held
  uint8_t kill;               // pinky or fist
};
GlovePacket gp;

float currentRoll = 0, currentPitch = 0;
bool gotImuData = false;
unsigned long lastSend = 0;

// tilt delta -> ±1000 with deadzone
int16_t tiltMap(float d){
  if(fabsf(d) < TILT_DEADZONE) return 0;
  float sign = (d > 0) ? 1.0f : -1.0f;
  float mag  = (fabsf(d) - TILT_DEADZONE) / (TILT_FULL - TILT_DEADZONE);
  if(mag > 1.0f) mag = 1.0f;
  return (int16_t)(sign * mag * 1000.0f);
}

void setup(){
  Serial.begin(115200);
  delay(500);

  pinMode(PINCH_INDEX,  INPUT_PULLUP);
  pinMode(PINCH_MIDDLE, INPUT_PULLUP);
  pinMode(PINCH_RING,   INPUT_PULLUP);
  pinMode(PINCH_PINKY,  INPUT_PULLUP);
  pinMode(HALL_INDEX,   INPUT_PULLUP);
  pinMode(HALL_RING,    INPUT_PULLUP);

  Wire.begin();
  delay(100);
  if(!bno08x.begin_I2C(0x4B)){
    Serial.println("BNO085 NOT FOUND");
  } else {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);   // 50Hz
    Serial.println("BNO085 ready");
  }

  WiFi.mode(WIFI_STA);
  delay(500);
  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init FAIL");
    while(1){ delay(1000); }
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, txMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if(esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Add peer FAIL");
    while(1){ delay(1000); }
  }

  Serial.print("Glove packet size: "); Serial.println(sizeof(GlovePacket));
  Serial.println("Glove ready - sending to TX");
}

void loop(){
  // ---- IMU ----
  if(bno08x.wasReset()) bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);

  if(bno08x.getSensorEvent(&imuValue)){
    if(imuValue.sensorId == SH2_ROTATION_VECTOR){
      float qw = imuValue.un.rotationVector.real;
      float qx = imuValue.un.rotationVector.i;
      float qy = imuValue.un.rotationVector.j;
      float qz = imuValue.un.rotationVector.k;

      float sinr = 2.0f*(qw*qx + qy*qz);
      float cosr = 1.0f - 2.0f*(qx*qx + qy*qy);
      currentRoll = atan2(sinr, cosr) * 57.2958f;

      float sinp = 2.0f*(qw*qy - qz*qx);
      if(sinp >  1.0f) sinp =  1.0f;
      if(sinp < -1.0f) sinp = -1.0f;
      currentPitch = asin(sinp) * 57.2958f;

      gotImuData = true;
    }
  }

  if(millis()-lastSend < 20){ delay(1); return; }   // 50Hz
  lastSend = millis();

  // ---- tilt -> stick (coordinate transform from the glove's mounting angle) ----
  float pitchDelta = currentPitch - PITCH_NEUTRAL;
  float rollDelta  = currentRoll  - ROLL_NEUTRAL;
  float strafe = pitchDelta + rollDelta;    // hand tilt left/right
  float walk   = pitchDelta - rollDelta;    // hand tilt fwd/back

  int16_t lateral = tiltMap(strafe);
  bool yawMode = (digitalRead(PINCH_RING) == LOW);

  if(yawMode){
    gp.roll = 0;              // no roll while turning
    gp.yaw  = lateral;        // right tilt = CW
  } else {
    gp.roll = lateral;
    gp.yaw  = 0;
  }
  gp.pitch  = tiltMap(walk);
  gp.active = gotImuData ? 1 : 0;

  gp.thrUp = (digitalRead(PINCH_INDEX)  == LOW) ? 1 : 0;
  gp.land  = (digitalRead(PINCH_MIDDLE) == LOW) ? 1 : 0;

  bool pinky = (digitalRead(PINCH_PINKY) == LOW);
  bool fist  = (digitalRead(HALL_INDEX)==LOW) && (digitalRead(HALL_RING)==LOW);
  gp.kill = (pinky || fist) ? 1 : 0;

  esp_now_send(txMac, (uint8_t*)&gp, sizeof(gp));

  static unsigned long lastPrint = 0;
  if(millis()-lastPrint > 200){
    lastPrint = millis();
    Serial.printf("P:%+5d R:%+5d Y:%+5d act:%u thr:%u land:%u KILL:%u\n",
      gp.pitch, gp.roll, gp.yaw, gp.active, gp.thrUp, gp.land, gp.kill);
  }
}