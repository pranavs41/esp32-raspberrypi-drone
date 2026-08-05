#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DShotRMT.h>
#include <esp_wifi.h>


#define MOTORS_LIVE 1
#define PITCH_ANGLE_SIGN  -1
#define ROLL_ANGLE_SIGN   +1


#define USE_FIXED_LEVEL 1
const float PITCH_OFFSET_FIXED =  -1.5f;
const float ROLL_OFFSET_FIXED  = -1.05f; //-1.35  CONVERGED: iR plateaued ~5.2 over 20s hover
float YAW_TRIM   = -2.0f;
float PITCH_TRIM = 6.0f;


struct __attribute__((packed)) Packet { uint16_t throttle,yaw,pitch,roll; uint8_t arm; };
Packet rx;
volatile bool packetFresh = false;
Packet rxBuf;
unsigned long lastReceive=0;


// 2026-08-02: flow fields + hdg added - MUST match TX byte-for-byte (37 bytes)
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

// ---- DEBUG: byte counter (starves the parser - fF stays 0 while this is 1) ----
#define FLOW_RAW_DEBUG 0
// ---- M4: optical flow velocity damping ----
// SIGN VERIFIED 2026-08: slide left -> fVx positive -> fRA positive -> right lean
//                        -> opposes drift. No negation needed.
#define VEL_GAIN     0.375f   // deg lean per (px/frame) //was 0.375
#define VEL_Q_MIN    25      // min flow quality to trust the data
#define VEL_MAX_ANG  3.25f    // hard clamp on flow-commanded lean
#define VEL_ENABLE   1       // 0 = fly with damping off
#define VEL_KD       0.06f   // damping on rate-of-change of flow
#define VEL_D_MAX    2.0f    // clamp on the D contribution alone
// ---- altitude hold ----
#define ALT_ENABLE     1
#define ALT_KP         125.0f   // throttle units per metre of error
#define ALT_KI          40.0f   // throttle units per metre-second
#define ALT_KD   180.0f
#define ALT_I_MAX      80.0f   // clamp on altitude integral
#define ALT_CLIMB_RATE   0.4f   // m/s max target climb
#define ALT_DESC_RATE    0.2f   // m/s max target descent (deliberately slower)
#define ALT_MIN_HOLD     0.40f  // don't engage below this
#define ALT_MAX_HOLD     2.50f  // don't engage above this
// ---- altitude-aware landing ----
#define LAND_SLOW_ALT   0.35f   // below this, descend gently
#define LAND_RATE_HIGH 180.0f   // throttle units/s above LAND_SLOW_ALT
#define LAND_RATE_LOW   15.0f   // throttle units/s in the slow zone
#define LAND_TOUCH_ALT  0.04f   // treat as touchdown below this
#define LAND_TOUCH_MS    250    // ...held this long
#define LAND_RATE_FAST  -0.35f   // m/s target descent above LAND_SLOW_ALT
#define LAND_RATE_SLOW  -0.12f   // m/s below it
#define LAND_KD          400.0f  // throttle per (m/s) of rate error

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  if(len == sizeof(Packet)){
    memcpy((void*)&rxBuf, data, sizeof(Packet));
    packetFresh = true;
    memcpy(txMac, info->src_addr, 6);
    txKnown = true;
  }
}


Adafruit_MPU6050 mpu;
float gyroBiasX=0,gyroBiasY=0,gyroBiasZ=0;
float pitchAngle=0, rollAngle=0, pitchOffset=0, rollOffset=0;
float headingRel = 0;           // deg from arming heading
float altM = -1.0f;             // metres from lidar, -1 = invalid
bool  altOk = false;
unsigned long lastAltMs = 0;

float altTarget = 0;            // <<< NEW
float altI = 0;                 // <<< NEW
float altBase = 0;      // throttle at the moment hold engaged
float altPrev = 0;      // previous altitude, for climb rate
unsigned long lastAltCalc = 0;
bool  altHoldActive = false;    // <<< NEW
unsigned long touchSince = 0;

DShotRMT m1(GPIO_NUM_32,DSHOT300,false);
DShotRMT m2(GPIO_NUM_33,DSHOT300,false);
DShotRMT m3(GPIO_NUM_25,DSHOT300,false);
DShotRMT m4(GPIO_NUM_4, DSHOT300,false);


const int THR_CENTER=2128,YAW_CENTER=1909,PITCH_CENTER=2173,ROLL_CENTER=1968;
const int THR_DEADBAND=150;
const float THR_RATE=0.0007f;
const int STICK_DEADBAND=150;
const int YAW_DEADBAND=4096;   // intentionally > full ADC range: yaw stick disabled
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
const float ANGLE_KP=3.0f;


float Kp=1.2f, Kd=0.0f;
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
  altBase = 0; altPrev = 0;
  headingRel = 0;
  altI = 0; altHoldActive = false; touchSince = 0;   // <<< ADD
  prevRollErr=0; prevPitchErr=0;
  rollPauseUntil=0; pitchPauseUntil=0;
  iActive=false; iBelowSince=0;
}


void calibrate(){
  Serial.println("Calibrating - HOLD STILL...");
  const int N=1000; float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;
  for(int i=0;i<N;i++){ sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    gx+=g.gyro.x;gy+=g.gyro.y;gz+=g.gyro.z;
    ax+=a.acceleration.x;ay+=a.acceleration.y;az+=a.acceleration.z; delay(2); }
  gx/=N;gy/=N;gz/=N;ax/=N;ay/=N;az/=N;
  gyroBiasX=gx;gyroBiasY=gy;gyroBiasZ=gz;
#if USE_FIXED_LEVEL
  pitchOffset=PITCH_OFFSET_FIXED;
  rollOffset =ROLL_OFFSET_FIXED;
  Serial.printf("Level ref (FIXED): pitch=%.1f roll=%.1f\n",pitchOffset,rollOffset);
#else
  pitchOffset=atan2(ay,az)*57.2958f;
  rollOffset =atan2(ax,az)*57.2958f;
  Serial.printf("Level ref (measured): pitch=%.1f roll=%.1f\n",pitchOffset,rollOffset);
#endif
  pitchAngle = atan2(ay,az)*57.2958f - pitchOffset;
  rollAngle  = atan2(ax,az)*57.2958f - rollOffset;
  Serial.printf("Estimate init: pitch=%.1f roll=%.1f\n", pitchAngle, rollAngle);
}


void motorsOff(){ m1.sendThrottle(0);m2.sendThrottle(0);m3.sendThrottle(0);m4.sendThrottle(0); }


void setup(){
  Serial.begin(115200); delay(200);
  Serial2.begin(115200, SERIAL_8N1, 27, 17);
  Wire.begin(21,22);
  Wire.setClock(400000);
  if(!mpu.begin()){ Serial.println("MPU NOT found"); while(1){} }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  calibrate();
  m1.begin();m2.begin();m3.begin();m4.begin();
  for(int i=0;i<300;i++){ motorsOff(); delay(2); }


  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init() != ESP_OK){ Serial.println("ESP-NOW init FAIL"); while(1){} }
  esp_now_register_recv_cb(onRecv);
  Serial.print("FC MAC: "); Serial.println(WiFi.macAddress());

  Serial.print("Telem size: "); Serial.println(sizeof(Telem));   // must print 37

  Serial.printf("ANGLE MODE. FLOW_RAW_DEBUG=%d\n", FLOW_RAW_DEBUG);
  lastLoop=micros();
}


void loop(){

#if FLOW_RAW_DEBUG
  // ---- byte counter: drains the port itself, nothing can slip past ----
  static unsigned long dbgT=0;
  static unsigned long byteCount=0;
  static uint8_t firstBytes[16];
  static int firstIdx=0;

  while(Serial2.available()){
    uint8_t c = Serial2.read();
    byteCount++;
    if(firstIdx<16) firstBytes[firstIdx++]=c;
  }

  if(millis()-dbgT>1000){
    dbgT=millis();
    Serial.printf(">> bytes/sec:%lu  first16: ", byteCount);
    for(int i=0;i<firstIdx;i++) Serial.printf("%02X ", firstBytes[i]);
    Serial.println();
    byteCount=0;
  }
#endif

  unsigned long now=micros(); float dt=(now-lastLoop)/1000000.0f;
  if(dt<0.002f) return; lastLoop=now;


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
        // flowBuf[8] = seq
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


  if(linkOk && rx.arm==1 && !armed){
    armed=true; isSoftLanding=false;
    throttleHold=0; resetI();
    Serial.println(">>> ARMED");
  }
  if(linkOk && rx.arm==0 && armed){
    armed=false; isSoftLanding=false;
    throttleHold=0; resetI();
    Serial.println(">>> DISARMED");
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
          float want = (altM > LAND_SLOW_ALT) ? LAND_RATE_FAST : LAND_RATE_SLOW;
          static float dAlt = 0;                   // held between lidar samples
    if(lastAltMs != lastAltCalc){
      float adt = (lastAltMs - lastAltCalc) / 1000.0f;
      if(adt > 0.01f && adt < 0.5f) dAlt = (altM - altPrev) / adt;
      altPrev = altM;
      lastAltCalc = lastAltMs;
    }
          throttleHold += (want - dAlt) * LAND_KD * dt;
          throttleHold = constrain(throttleHold, MOTOR_IDLE, THR_MAX);

          if(altM < LAND_TOUCH_ALT){
            if(touchSince == 0) touchSince = millis();
            if(millis() - touchSince > LAND_TOUCH_MS){
              armed=false; throttleHold=0; touchSince=0; resetI();
              Serial.println(">>> TOUCHDOWN - DISARMED");
            }
          } else touchSince = 0;
        } else {
          throttleHold -= PILOT_LAND_RATE*dt;      // no lidar: old behaviour
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
      altBase   = throttleHold;    // hover throttle, captured on engage
      altPrev   = altM;
      altI      = 0;
    }
    float err  = altTarget - altM;
    float dAlt = (altM - altPrev) / dt;      // m/s, positive = climbing
    altPrev = altM;

    altI += err * dt;
    altI = constrain(altI, -1.5f, 1.5f);

    float corr = err*ALT_KP + altI*ALT_KI - dAlt*ALT_KD;
    corr = constrain(corr, -200.0f, 200.0f);
    throttleHold = constrain(altBase + corr, 0, THR_MAX);
  } else {
    altHoldActive = false;
  }
#endif

  sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);


  float accMag = sqrtf(a.acceleration.x*a.acceleration.x +
                       a.acceleration.y*a.acceleration.y +
                       a.acceleration.z*a.acceleration.z);
  accMagFilt = 0.999f*accMagFilt + 0.001f*accMag;
  float vib = accMag - accMagFilt;
  vibRMS = 0.995f*vibRMS + 0.005f*(vib*vib);
  dtMsFilt = 0.99f*dtMsFilt + 0.01f*(dt*1000.0f);


  float gPitchRate=(g.gyro.x-gyroBiasX)*57.2958f;
  float gRollRate =(g.gyro.y-gyroBiasY)*57.2958f;
  float gYaw      =(g.gyro.z-gyroBiasZ)*57.2958f;
  if(armed) headingRel += gYaw * dt;


  float pitchAccel=atan2(a.acceleration.y,a.acceleration.z)*57.2958f - pitchOffset;
  float rollAccel =atan2(a.acceleration.x,a.acceleration.z)*57.2958f - rollOffset;
  pitchAngle=0.998f*(pitchAngle+gPitchRate*dt)+0.002f*pitchAccel;
  rollAngle =0.998f*(rollAngle +gRollRate *dt)+0.002f*rollAccel;


  int yR = (int)rx.yaw   - YAW_CENTER;   if(abs(yR) < YAW_DEADBAND)   yR = 0;
  int pR = (int)rx.pitch - PITCH_CENTER; if(abs(pR) < STICK_DEADBAND) pR = 0;
  int rR = (int)rx.roll  - ROLL_CENTER;  if(abs(rR) < STICK_DEADBAND) rR = 0;
  float pitchSet = -pR/2048.0f*MAX_ANGLE;
  float rollSet  =  rR/2048.0f*MAX_ANGLE;   
  float setYaw   = -yR/2048.0f*MAX_YAWRATE;

// ---- heading hold: stick slews target, P term holds it ----
  #define HDG_KP 1.5f
  #define HDG_MAX_RATE 60.0f
  if(armed){
    if(yR != 0){
      // stick input: command rate directly, and drag the reference along
      headingRel = 0;              // stick-commanded yaw doesn't count as error
    } else {
      setYaw += constrain(-headingRel * HDG_KP, -HDG_MAX_RATE, HDG_MAX_RATE);
    }
  }

// ---- M4: flow damping - P on velocity, D on its rate of change ----
  float flowRollAdj = 0, flowPitchAdj = 0;
#if VEL_ENABLE
  if(flowFresh && flowQ >= VEL_Q_MIN && rR==0 && pR==0 && throttleHold > I_LIFT_THR){
    float fdt = (millis() - prevFlowMs) / 1000.0f;
    float dX = 0, dY = 0;
    if(fdt > 0.01f && fdt < 0.5f){          // sane interval only
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
    prevFlowMs = 0;      // force a clean restart when the gate reopens
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
    tl.alt = altOk ? altM : -1.0f;        // <<< ADD THIS LINE
    tl.thr = (uint16_t)throttleHold; tl.armd = armed ? 1 : 0;
    tl.fQ  = flowQ;
    tl.fF  = flowFresh ? 1 : 0;
    esp_now_send(txMac, (uint8_t*)&tl, sizeof(tl));
  }


  if(millis()-lastPrint>200){
    lastPrint=millis();
    const char* lnk = linkOk ? "OK" : (linkLost ? "LOST" : "GRACE");
      Serial.printf("%s thr:%4.0f P:%5.1f R:%5.1f vib:%4.2f iR:%5.1f iP:%5.1f fVx:%6.2f fVy:%6.2f fQ:%3u fF:%d fRA:%5.2f fPA:%5.2f hdg:%+6.1f alt:%5.2f LNK:%s\n",
      armed?"ARM":"DIS", throttleHold, pitchAngle, rollAngle, sqrtf(vibRMS),
      iRoll, iPitch, flowVelX, flowVelY, flowQ, flowFresh, flowRollAdj, flowPitchAdj, headingRel,
      altOk?altM:-1.0f, lnk);
  }
}