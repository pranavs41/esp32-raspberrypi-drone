#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <DShotRMT.h>
#include <esp_wifi.h>
#include <esp_mac.h>


#define MOTORS_LIVE 1
#define PITCH_ANGLE_SIGN  -1
#define ROLL_ANGLE_SIGN   +1

#define BNO_ADDR 0x4B

// AXIS MAPPING (measured 2026-08 on the BNO085):
//   nose down -> quaternion pitch POSITIVE  -> negate for nose-up-positive
//   right down -> quaternion roll POSITIVE  -> matches existing convention
//   CW yaw    -> gz NEGATIVE                -> negate for CW-positive
//   gx = roll rate (right positive), gy = pitch rate (nose down positive), gz = yaw

#define USE_FIXED_LEVEL 1
const float PITCH_OFFSET_FIXED =  1.8f;   // RE-VERIFY: BNO mount differs from MPU
const float ROLL_OFFSET_FIXED  = 2.1f;   // RE-VERIFY
float YAW_TRIM   = -2.0f;
float PITCH_TRIM = 13.0f;


struct __attribute__((packed)) Packet { uint16_t throttle,yaw,pitch,roll; uint8_t arm; };
Packet rx;
volatile bool packetFresh = false;
Packet rxBuf;
unsigned long lastReceive=0;


struct __attribute__((packed)) Telem {
  float pitch, roll, iR, iP, vib;
  float fVx, fVy;
  float hdg;
  float alt;
  uint16_t thr;
  uint8_t  armd, fQ, fF;
};
uint8_t txMac[6];
volatile bool txKnown=false;
bool peerAdded=false;
unsigned long lastTelem=0;


// ---- Pi optical flow link ----
float flowVelX=0, flowVelY=0;
uint8_t flowQ=0;
bool flowValid=false;
unsigned long lastFlowMs=0;
uint8_t flowBuf[16];
int flowIdx=0;
float prevFlowX = 0, prevFlowY = 0;
unsigned long prevFlowMs = 0;

#define FLOW_RAW_DEBUG 0

// ---- M4: optical flow velocity damping ----
#define VEL_GAIN     0.375f
#define VEL_Q_MIN    25
#define VEL_MAX_ANG  3.25f
#define VEL_ENABLE   1
#define VEL_KD       0.06f
#define VEL_D_MAX    2.0f

// ---- altitude hold ----
#define ALT_ENABLE     1
#define ALT_KP         120.0f
#define ALT_KI          40.0f
#define ALT_KD         180.0f
#define ALT_MIN_HOLD     0.40f
#define ALT_MAX_HOLD     2.50f

// ---- landing (slow bleed to a floor) ----
#define LAND_BLEED      12.0f
#define LAND_FLOOR     660.0f
#define LAND_TOUCH_ALT   0.04f
#define LAND_TOUCH_MS    250

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if(len == sizeof(Packet)){
    memcpy((void*)&rxBuf, data, sizeof(Packet));
    packetFresh = true;
    memcpy(txMac, info->src_addr, 6);
    txKnown = true;
  }
}


Adafruit_BNO08x bno;
sh2_SensorValue_t bnoVal;
bool imuOk = false;
unsigned long lastImuMs = 0;

float pitchAngle=0, rollAngle=0, pitchOffset=0, rollOffset=0;
float gPitchRate=0, gRollRate=0, gYaw=0;
float accX=0, accY=0, accZ=9.81f;
float headingRel = 0;

float altM = -1.0f;
bool  altOk = false;
unsigned long lastAltMs = 0;

float altTarget = 0;
float altI = 0;
float altBase = 0;
float altPrev = 0;
unsigned long lastAltCalc = 0;
bool  altHoldActive = false;
unsigned long touchSince = 0;

DShotRMT m1(GPIO_NUM_32,DSHOT300,false);
DShotRMT m2(GPIO_NUM_33,DSHOT300,false);
DShotRMT m3(GPIO_NUM_25,DSHOT300,false);
DShotRMT m4(GPIO_NUM_4, DSHOT300,false);


const int THR_CENTER=2128,YAW_CENTER=1909,PITCH_CENTER=2173,ROLL_CENTER=1968;
const int THR_DEADBAND=150;
const float THR_RATE=0.0015f;
const int STICK_DEADBAND=150;
const int YAW_DEADBAND=4096;
const int THR_MAX=1800;
const int THR_GATE=100;
const int MOTOR_IDLE=80;


#define GRACE_MS     350
#define FAILSAFE_MS  700
#define LANDING_RATE 120.0f
#define LAND_STICK_THR   300
#define PILOT_LAND_RATE  180.0f


float throttleHold=0;

const float MAX_ANGLE=15.0f;
const float MAX_YAWRATE=200.0f;
const float ANGLE_KP=2.75f;

float Kp=1.0f, Kd=0.0f;
float Ki_roll=0.35f, Ki_pitch=0.35f, Ki_yaw=0.20f;
float iRoll=0,iPitch=0,iYaw=0, ePrevRoll=0,ePrevPitch=0,ePrevYaw=0;

#define I_LIFT_THR   550.0f
#define I_LAND_THR   350.0f
#define I_LAND_MS    400
bool iActive=false;
unsigned long iBelowSince=0;

#define I_PAUSE_MS 60
#define I_CROSS_DECAY 0.85f
#define I_CROSS_BAND 0.5f
#define I_RATE_GATE  60.0f
#define I_LEAK_TC    240.0f

bool armed=false;
unsigned long lastLoop=0,lastPrint=0;

float accMagFilt = 9.81f, vibRMS = 0, dtMsFilt = 2.0f;

unsigned long rollPauseUntil = 0;
unsigned long pitchPauseUntil = 0;
float prevRollErr = 0;
float prevPitchErr = 0;


void resetI(){
  iRoll=0; iPitch=0; iYaw=0;
  headingRel = 0;
  altI = 0; altHoldActive = false; touchSince = 0;
  altBase = 0; altPrev = 0; lastAltCalc = 0;
  prevRollErr=0; prevPitchErr=0;
  rollPauseUntil=0; pitchPauseUntil=0;
  iActive=false; iBelowSince=0;
}


void enableReports(){
  bno.enableReport(SH2_ROTATION_VECTOR, 10000);        // 200Hz
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  bno.enableReport(SH2_ACCELEROMETER, 20000);         // 50Hz, unchanged
}


void motorsOff(){ m1.sendThrottle(0);m2.sendThrottle(0);m3.sendThrottle(0);m4.sendThrottle(0); }


void setup(){
  Serial.begin(115200); delay(200);
  Serial2.begin(115200, SERIAL_8N1, 27, 17);

  Wire.begin(21,22);
  Wire.setClock(100000);      // BNO08x stretches the clock - 400k is unreliable on ESP32
  Wire.setTimeOut(10);
  delay(200);

  uint8_t addrs[2] = {0x4B, 0x4A};
  for(int i=0; i<10 && !imuOk; i++){
    uint8_t a = addrs[i % 2];
    if(bno.begin_I2C(a)){
      imuOk = true;
      enableReports();
      Serial.printf("BNO085 ready at 0x%02X\n", a);
    } else {
      Serial.printf("BNO try %d at 0x%02X failed\n", i+1, a);
      delay(400);
    }
  }
  if(!imuOk){ Serial.println("BNO NOT FOUND"); while(1){ delay(1000); } }

  pitchOffset = PITCH_OFFSET_FIXED;
  rollOffset  = ROLL_OFFSET_FIXED;
  Serial.printf("Level ref (FIXED): pitch=%.1f roll=%.1f\n", pitchOffset, rollOffset);

  m1.begin();m2.begin();m3.begin();m4.begin();
  for(int i=0;i<300;i++){ motorsOff(); delay(2); }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init() != ESP_OK){ Serial.println("ESP-NOW init FAIL"); while(1){} }
  esp_now_register_recv_cb(onRecv);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("FC MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.print("Telem size: "); Serial.println(sizeof(Telem));
  Serial.printf("ANGLE MODE (BNO085). FLOW_RAW_DEBUG=%d\n", FLOW_RAW_DEBUG);
  lastLoop=micros();
}


void loop(){
  unsigned long now=micros(); float dt=(now-lastLoop)/1000000.0f;
  if(dt<0.002f) return; lastLoop=now;

  // ---- IMU ----
  if(bno.wasReset()) enableReports();

  while(bno.getSensorEvent(&bnoVal)){
    if(bnoVal.sensorId == SH2_ROTATION_VECTOR){
      float qw=bnoVal.un.rotationVector.real, qx=bnoVal.un.rotationVector.i;
      float qy=bnoVal.un.rotationVector.j,    qz=bnoVal.un.rotationVector.k;

      float r = atan2(2.0f*(qw*qx+qy*qz), 1.0f-2.0f*(qx*qx+qy*qy)) * 57.2958f;
      float sp = 2.0f*(qw*qy-qz*qx);
      if(sp> 1.0f) sp= 1.0f;
      if(sp<-1.0f) sp=-1.0f;
      float p = asin(sp) * 57.2958f;

      rollAngle  =  r - rollOffset;      // right down = positive: matches
      pitchAngle = -p - pitchOffset;     // nose down was positive: negate
      lastImuMs = millis();
    }
    else if(bnoVal.sensorId == SH2_GYROSCOPE_CALIBRATED){
      gRollRate  =  bnoVal.un.gyroscope.x * 57.2958f;   // right positive: matches
      gPitchRate = -bnoVal.un.gyroscope.y * 57.2958f;   // nose down was positive: negate
      gYaw       = bnoVal.un.gyroscope.z * 57.2958f;   // CW was negative: negate
    }
    else if(bnoVal.sensorId == SH2_ACCELEROMETER){
      accX = bnoVal.un.accelerometer.x;
      accY = bnoVal.un.accelerometer.y;
      accZ = bnoVal.un.accelerometer.z;
    }
  }

  bool imuFresh = (millis() - lastImuMs) < 100;

#if FLOW_RAW_DEBUG
  static unsigned long dbgT=0, byteCount=0;
  while(Serial2.available()){ Serial2.read(); byteCount++; }
  if(millis()-dbgT>1000){ dbgT=millis();
    Serial.printf(">> bytes/sec:%lu\n", byteCount); byteCount=0; }
#endif

  if(packetFresh){
    packetFresh = false;
    Packet tmp = rxBuf;
    if(tmp.throttle<=4095 && tmp.yaw<=4095 && tmp.pitch<=4095 && tmp.roll<=4095
       && tmp.arm<=1
       && !(tmp.throttle==0 && tmp.yaw==0 && tmp.pitch==0 && tmp.roll==0)){
      rx = tmp;
      lastReceive = millis();
    }
  }

  // ---- read Pi flow packets ----
  while(Serial2.available()){
    uint8_t c = Serial2.read();
    if(flowIdx==0 && c!=0xAA) continue;
    if(flowIdx==1 && c!=0x55){ flowIdx=0; continue; }
    flowBuf[flowIdx++]=c;
    if(flowIdx==12){
      flowIdx=0;
      uint8_t ck=0;
      for(int i=2;i<11;i++) ck^=flowBuf[i];
      if(ck==flowBuf[11]){
        int16_t vx = flowBuf[2] | (flowBuf[3]<<8);
        int16_t vy = flowBuf[4] | (flowBuf[5]<<8);
        flowVelX = vx/100.0f;
        flowVelY = vy/100.0f;
        flowQ    = flowBuf[6];
        flowValid= flowBuf[7] & 1;
        uint16_t acm = flowBuf[9] | (flowBuf[10]<<8);
        if(acm > 0 && acm < 400){ altM = acm/100.0f; altOk = true; lastAltMs = millis(); }
        else altOk = false;
        lastFlowMs = millis();
      }
    }
  }
  bool flowFresh = (millis()-lastFlowMs) < 200;
  if(millis() - lastAltMs > 300) altOk = false;

  if(txKnown && !peerAdded){
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, txMac, 6);
    p.channel = 0; p.encrypt = false;
    if(esp_now_add_peer(&p) == ESP_OK){
      peerAdded = true;
      Serial.println(">>> TELEM PEER ADDED");
    }
  }

  unsigned long linkAge = millis()-lastReceive;
  bool linkOk   = linkAge < GRACE_MS;
  bool linkLost = linkAge > FAILSAFE_MS;

  static bool isSoftLanding = false;

  if(linkOk && rx.arm==1 && !armed && imuFresh){
    armed=true; isSoftLanding=false;
    throttleHold=0; resetI();
    Serial.println(">>> ARMED");
  }
  if(linkOk && rx.arm==0 && armed){
    armed=false; isSoftLanding=false;
    throttleHold=0; resetI();
    Serial.println(">>> DISARMED");
  }
  if(armed && !imuFresh){
    armed=false; throttleHold=0; resetI();
    Serial.println(">>> IMU STALE - DISARMED");
  }
  if(linkLost && armed && !isSoftLanding){
    isSoftLanding=true;
    Serial.println(">>> LINK LOST - SOFT LANDING...");
  }
  if(linkOk && rx.arm==1 && armed && isSoftLanding){
    isSoftLanding=false;
    Serial.println(">>> LINK RESTORED - LANDING ABORTED");
  }

  if(armed){
    if(isSoftLanding){
      throttleHold -= LANDING_RATE*dt;
      if(throttleHold <= MOTOR_IDLE){
        armed=false; isSoftLanding=false;
        throttleHold=0; resetI();
        Serial.println(">>> FAILSAFE LANDED & DISARMED");
      }
    } else if(linkOk){
      bool landStick = (rx.throttle < LAND_STICK_THR);

      if(landStick && throttleHold > MOTOR_IDLE){
        if(altOk){
          throttleHold -= LAND_BLEED*dt;
          if(altM > LAND_TOUCH_ALT && throttleHold < LAND_FLOOR)
            throttleHold = LAND_FLOOR;
          throttleHold = constrain(throttleHold, MOTOR_IDLE, THR_MAX);

          if(altM < LAND_TOUCH_ALT){
            if(touchSince == 0) touchSince = millis();
            if(millis() - touchSince > LAND_TOUCH_MS){
              armed=false; throttleHold=0; touchSince=0; resetI();
              Serial.println(">>> TOUCHDOWN - DISARMED");
            }
          } else touchSince = 0;
        } else {
          throttleHold -= PILOT_LAND_RATE*dt;
          if(throttleHold <= MOTOR_IDLE){
            armed=false; throttleHold=0; touchSince=0; resetI();
            Serial.println(">>> PILOT LANDED & DISARMED");
          }
        }
      } else if(!landStick){
        int d=(int)rx.throttle-THR_CENTER;
        if(abs(d)>THR_DEADBAND){
          throttleHold += d*THR_RATE;
          throttleHold = constrain(throttleHold,0,THR_MAX);
        }
      }
    }
  }

// ---- altitude hold ----
#if ALT_ENABLE
  bool thrStick = (abs((int)rx.throttle - THR_CENTER) > THR_DEADBAND);
  bool altCanHold = armed && linkOk && !isSoftLanding && altOk
                    && altM > ALT_MIN_HOLD && altM < ALT_MAX_HOLD
                    && throttleHold > I_LIFT_THR;

  if(altCanHold && !thrStick){
    if(!altHoldActive){
      altHoldActive = true;
      altTarget = altM;
      altBase   = throttleHold;
      altPrev   = altM;
      altI      = 0;
    }
    static float dAlt = 0;
    if(lastAltMs != lastAltCalc){
      float adt = (lastAltMs - lastAltCalc) / 1000.0f;
      if(adt > 0.01f && adt < 0.5f) dAlt = (altM - altPrev) / adt;
      altPrev = altM;
      lastAltCalc = lastAltMs;
    }
    float err = altTarget - altM;
    altI += err * dt;
    altI = constrain(altI, -1.5f, 1.5f);
    float corr = err*ALT_KP + altI*ALT_KI - dAlt*ALT_KD;
    corr = constrain(corr, -200.0f, 200.0f);
    throttleHold = constrain(altBase + corr, 0, THR_MAX);
  } else {
    altHoldActive = false;
  }
#endif

  // ---- vibration metric (from BNO accelerometer) ----
  float accMag = sqrtf(accX*accX + accY*accY + accZ*accZ);
  accMagFilt = 0.999f*accMagFilt + 0.001f*accMag;
  float vib = accMag - accMagFilt;
  vibRMS = 0.995f*vibRMS + 0.005f*(vib*vib);
  dtMsFilt = 0.99f*dtMsFilt + 0.01f*(dt*1000.0f);

  if(armed) headingRel += gYaw * dt;

  int yR = (int)rx.yaw   - YAW_CENTER;   if(abs(yR) < YAW_DEADBAND)   yR = 0;
  int pR = (int)rx.pitch - PITCH_CENTER; if(abs(pR) < STICK_DEADBAND) pR = 0;
  int rR = (int)rx.roll  - ROLL_CENTER;  if(abs(rR) < STICK_DEADBAND) rR = 0;
  float pitchSet = -pR/2048.0f*MAX_ANGLE;
  float rollSet  =  rR/2048.0f*MAX_ANGLE;
  float setYaw   = -yR/2048.0f*MAX_YAWRATE;

  // ---- heading hold ----
  #define HDG_KP 1.5f
  #define HDG_MAX_RATE 60.0f
  if(armed){
    if(yR != 0){
      headingRel = 0;
    } else {
      setYaw += constrain(-headingRel * HDG_KP, -HDG_MAX_RATE, HDG_MAX_RATE);
    }
  }

  // ---- M4: flow damping ----
  float flowRollAdj = 0, flowPitchAdj = 0;
#if VEL_ENABLE
  if(flowFresh && flowQ >= VEL_Q_MIN && rR==0 && pR==0 && throttleHold > I_LIFT_THR){
    float fdt = (millis() - prevFlowMs) / 1000.0f;
    float dX = 0, dY = 0;
    if(fdt > 0.01f && fdt < 0.5f){
      dX = (flowVelX - prevFlowX) / fdt;
      dY = (flowVelY - prevFlowY) / fdt;
    }
    prevFlowX = flowVelX; prevFlowY = flowVelY; prevFlowMs = millis();

    float dRollAdj  = constrain(dX * VEL_KD, -VEL_D_MAX, VEL_D_MAX);
    float dPitchAdj = constrain(dY * VEL_KD, -VEL_D_MAX, VEL_D_MAX);

    flowRollAdj  = constrain(flowVelX*VEL_GAIN + dRollAdj,  -VEL_MAX_ANG, VEL_MAX_ANG);
    flowPitchAdj = constrain(flowVelY*VEL_GAIN + dPitchAdj, -VEL_MAX_ANG, VEL_MAX_ANG);
    rollSet  += flowRollAdj;
    pitchSet += flowPitchAdj;
  } else {
    prevFlowMs = 0;
  }
#endif

  float setPitch = ANGLE_KP * (pitchSet - pitchAngle) * PITCH_ANGLE_SIGN;
  float setRoll  = ANGLE_KP * (rollSet  - rollAngle ) * ROLL_ANGLE_SIGN;

  float pRate = gPitchRate * PITCH_ANGLE_SIGN;
  float rRate = gRollRate  * ROLL_ANGLE_SIGN;

  float eRoll=setRoll-rRate, ePitch=setPitch-pRate, eYaw=setYaw-gYaw;

  float rollErr = rollSet - rollAngle;
  float pitchErr = pitchSet - pitchAngle;

  if(!iActive && throttleHold > I_LIFT_THR) iActive = true;
  if(iActive){
    if(throttleHold < I_LAND_THR){
      if(iBelowSince == 0) iBelowSince = millis();
      if(millis() - iBelowSince > I_LAND_MS) iActive = false;
    } else iBelowSince = 0;
  }

  if(iActive){
    if(rollErr * prevRollErr < 0 &&
       fabsf(rollErr) > I_CROSS_BAND && fabsf(prevRollErr) > I_CROSS_BAND){
      iRoll *= I_CROSS_DECAY;
      rollPauseUntil = millis() + I_PAUSE_MS;
    }
    if(pitchErr * prevPitchErr < 0 &&
       fabsf(pitchErr) > I_CROSS_BAND && fabsf(prevPitchErr) > I_CROSS_BAND){
      iPitch *= I_CROSS_DECAY;
      pitchPauseUntil = millis() + I_PAUSE_MS;
    }

    bool rollStick  = (rR != 0);
    bool pitchStick = (pR != 0);

    static bool prevRollStick=false, prevPitchStick=false;
    if(prevRollStick && !rollStick){
      iRoll *= I_CROSS_DECAY;
      rollPauseUntil = millis() + I_PAUSE_MS;
    }
    if(prevPitchStick && !pitchStick){
      iPitch *= I_CROSS_DECAY;
      pitchPauseUntil = millis() + I_PAUSE_MS;
    }
    prevRollStick = rollStick;
    prevPitchStick = pitchStick;

    bool rollQuiet  = fabsf(gRollRate)  < I_RATE_GATE;
    bool pitchQuiet = fabsf(gPitchRate) < I_RATE_GATE;

    if(!rollStick  && rollQuiet  && millis() > rollPauseUntil)  iRoll  += rollErr  * dt;
    if(!pitchStick && pitchQuiet && millis() > pitchPauseUntil) iPitch += pitchErr * dt;

    iRoll  -= iRoll  * (dt / I_LEAK_TC);
    iPitch -= iPitch * (dt / I_LEAK_TC);

    iYaw += eYaw * dt;
    iRoll =constrain(iRoll,-60,60);
    iPitch=constrain(iPitch,-40,40);
    iYaw  =constrain(iYaw,-80,80);
  } else {
    iRoll=0; iPitch=0; iYaw=0;
    prevRollErr=0; prevPitchErr=0;
    rollPauseUntil=0; pitchPauseUntil=0;
  }

  prevRollErr = rollErr;
  prevPitchErr = pitchErr;

  float dR=(eRoll-ePrevRoll)/dt, dP=(ePitch-ePrevPitch)/dt, dY=(eYaw-ePrevYaw)/dt;
  ePrevRoll=eRoll; ePrevPitch=ePitch; ePrevYaw=eYaw;

  float outRoll  = Kp*eRoll  + Ki_roll *iRoll *ROLL_ANGLE_SIGN  + Kd*dR;
  float outPitch = Kp*ePitch + Ki_pitch*iPitch*PITCH_ANGLE_SIGN + Kd*dP;
  float outYaw   = Kp*eYaw   + Ki_yaw  *iYaw                     + Kd*dY;

  float base=throttleHold;
  float s1=base+outPitch+PITCH_TRIM-outRoll+outYaw-YAW_TRIM;
  float s2=base-outPitch-PITCH_TRIM-outRoll-outYaw+YAW_TRIM;
  float s3=base+outPitch+PITCH_TRIM+outRoll-outYaw+YAW_TRIM;
  float s4=base-outPitch-PITCH_TRIM+outRoll+outYaw-YAW_TRIM;

  int d1=0,d2=0,d3=0,d4=0;
  if(armed && throttleHold>THR_GATE){
    d1=constrain((int)s1,MOTOR_IDLE,1999)+48; d2=constrain((int)s2,MOTOR_IDLE,1999)+48;
    d3=constrain((int)s3,MOTOR_IDLE,1999)+48; d4=constrain((int)s4,MOTOR_IDLE,1999)+48;
  }

#if MOTORS_LIVE
  if(armed){ m1.sendThrottle(d1);m2.sendThrottle(d2);m3.sendThrottle(d3);m4.sendThrottle(d4); }
  else motorsOff();
#else
  motorsOff();
#endif

  if(peerAdded && millis()-lastTelem > 100){
    lastTelem = millis();
    Telem tl;
    tl.pitch = pitchAngle; tl.roll = rollAngle;
    tl.iR = iRoll; tl.iP = iPitch; tl.vib = sqrtf(vibRMS);
    tl.fVx = flowVelX;
    tl.fVy = flowVelY;
    tl.hdg = headingRel;
    tl.alt = altOk ? altM : -1.0f;
    tl.thr = (uint16_t)throttleHold; tl.armd = armed ? 1 : 0;
    tl.fQ  = flowQ;
    tl.fF  = flowFresh ? 1 : 0;
    esp_now_send(txMac, (uint8_t*)&tl, sizeof(tl));
  }

  if(millis()-lastPrint>200){
    lastPrint=millis();
    const char* lnk = linkOk ? "OK" : (linkLost ? "LOST" : "GRACE");
    Serial.printf("%s thr:%4.0f m:%4d/%4d/%4d/%4d P:%5.1f R:%5.1f vib:%4.2f iR:%5.1f iP:%5.1f fVx:%6.2f fVy:%6.2f fQ:%3u fF:%d fRA:%5.2f fPA:%5.2f hdg:%+6.1f alt:%5.2f imu:%d LNK:%s\n",
      armed?"ARM":"DIS", throttleHold, d1, d2, d3, d4, pitchAngle, rollAngle, sqrtf(vibRMS),
      iRoll, iPitch, flowVelX, flowVelY, flowQ, flowFresh, flowRollAdj, flowPitchAdj, headingRel,
      altOk?altM:-1.0f, imuFresh, lnk);
  }
}