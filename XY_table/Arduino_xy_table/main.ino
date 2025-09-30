#include <AccelStepper.h>
#include <math.h>

/* ===== RAMPS 1.4 pins (Mega2560) ===== */
#define X_STEP 46
#define X_DIR  48
#define X_EN   62
#define Y_STEP 60
#define Y_DIR  61
#define Y_EN   56

// Endstops (MIN only)
#define X_MIN_PIN 3     // X-MIN
#define Y_MIN_PIN 14    // Y-MIN

/* ===== ALM / PED inputs (active-LOW) =====
   X_ALM -> X-MAX (D2), X_PED -> Z-MAX (D19)
   Y_ALM -> Y-MAX (D15), Y_PED -> Z-MIN (D18)
*/
#define X_ALM_PIN  2
#define X_PED_PIN 19
#define Y_ALM_PIN 15
#define Y_PED_PIN 18

#define PED_X_ACTIVE_LOW 0
#define PED_Y_ACTIVE_LOW 0

/* ===== Relays (driver power) =====
   SERVO1(D4)=X, SERVO2(D5)=Y
   RELAY_ACTIVE_HIGH: 1 -> HIGH=ON, 0 -> LOW=ON
*/
#define RELAY_X_PIN 4
#define RELAY_Y_PIN 5
#define RELAY_ACTIVE_HIGH 0

/* ===== CONFIG ===== */
float STEPS_PER_MM_X = 20.0f;
float STEPS_PER_MM_Y = 20.0f;

float MAX_FEED_MM_S  = 600.0f;
float MAX_ACC_MM_S2  = 90000.0f;

float X_MIN_MM = 0.0f, X_MAX_MM = 165.0f;
float Y_MIN_MM = 0.0f, Y_MAX_MM = 350.0f;

float SCAN_RANGE_X_MM = 168.0f;
float SCAN_RANGE_Y_MM = 352.0f;
float BACKOFF_MM      = 5.0f;
float SLOW_MM_S       = 2.0f;

float WORK_X_MM = 5.0f;
float WORK_Y_MM = 350.0f;
float WORK_F_MM_MIN = 60000;

bool INVERT_X_DIR = true;
bool INVERT_Y_DIR = false;

bool X_ENDSTOP_ACTIVE_LOW = false;
bool Y_ENDSTOP_ACTIVE_LOW = false;

/* ===== Timings ===== */
#define DEBOUNCE_MS        10
#define RESET_OFF_MS       500
#define POWER_UP_WAIT_MS   1500
#define PED_WAIT_TIMEOUT   8000
#define PED_SETTLE_MS      50

/* ===== Geometry tolerances ===== */
float EPS_X_MM = 0.20f;
float EPS_Y_MM = 0.20f;

long EPS_X_STEPS = 0;
long EPS_Y_STEPS = 0;

/* ===== Globals ===== */
AccelStepper stepX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP, Y_DIR);

String ibuf;
bool estop=false;

/* ===== Endstop helper ===== */
inline bool endActive(uint8_t pin, bool activeLow){
  int v = digitalRead(pin);
  return activeLow ? (v==LOW) : (v==HIGH);
}

/* ===== Debounce channels for ALM/PED (no custom types) ===== */
bool AX_state=false, AX_lastRaw=false; uint32_t AX_tEdge=0; // X_ALM
bool AY_state=false, AY_lastRaw=false; uint32_t AY_tEdge=0; // Y_ALM
bool PX_state=false, PX_lastRaw=false; uint32_t PX_tEdge=0; // X_PED
bool PY_state=false, PY_lastRaw=false; uint32_t PY_tEdge=0; // Y_PED

static inline bool rawActiveRead(uint8_t pin, bool activeLow){
  int v = digitalRead(pin);
  return activeLow ? (v==LOW) : (v==HIGH);
}

static inline bool debounceReadPin(uint8_t pin, bool &state, bool &lastRaw, uint32_t &tEdge){
  bool rawActive = (digitalRead(pin) == LOW); // active-LOW
  if (rawActive != lastRaw){ lastRaw = rawActive; tEdge = millis(); }
  if ((uint32_t)(millis() - tEdge) > DEBOUNCE_MS) state = rawActive;
  return state;
}

inline bool alarmX(){ return debounceReadPin(X_ALM_PIN, AX_state, AX_lastRaw, AX_tEdge); }
inline bool alarmY(){ return debounceReadPin(Y_ALM_PIN, AY_state, AY_lastRaw, AY_tEdge); }
inline bool pedActiveX(){ return debounceReadPinPol(X_PED_PIN, PED_X_ACTIVE_LOW, PX_state, PX_lastRaw, PX_tEdge); }
inline bool pedActiveY(){ return debounceReadPinPol(Y_PED_PIN, PED_Y_ACTIVE_LOW, PY_state, PY_lastRaw, PY_tEdge); }

static inline bool debounceReadPinPol(uint8_t pin, bool activeLow,
                                      bool &state, bool &lastRaw, uint32_t &tEdge){
  bool rawActive = rawActiveRead(pin, activeLow);
  if (rawActive != lastRaw){ lastRaw = rawActive; tEdge = millis(); }
  if ((uint32_t)(millis() - tEdge) > DEBOUNCE_MS) state = rawActive;
  return state;
}

/* ===== Relays ===== */
inline void relayWrite(uint8_t pin, bool on){
  if (RELAY_ACTIVE_HIGH) digitalWrite(pin, on ? HIGH : LOW);
  else                   digitalWrite(pin, on ? LOW  : HIGH);
}
inline void powerX(bool on){ relayWrite(RELAY_X_PIN,on); }
inline void powerY(bool on){ relayWrite(RELAY_Y_PIN,on); }

/* ===== One-shot announce flags ===== */
bool motX_ok_ann = false, motY_ok_ann = false;
bool motX_alrm_ann = false, motY_alrm_ann = false;

/* ===== Motors ===== */
void motorsEnable(bool en){
  digitalWrite(X_EN, en?LOW:HIGH); // RAMPS: LOW=enable
  digitalWrite(Y_EN, en?LOW:HIGH);
}

void setupPins(){
  pinMode(X_EN, OUTPUT); pinMode(Y_EN, OUTPUT);
  motorsEnable(true);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);

  pinMode(X_ALM_PIN, INPUT_PULLUP);
  pinMode(Y_ALM_PIN, INPUT_PULLUP);
  pinMode(X_PED_PIN, INPUT_PULLUP);
  pinMode(Y_PED_PIN, INPUT_PULLUP);

  pinMode(RELAY_X_PIN, OUTPUT);
  pinMode(RELAY_Y_PIN, OUTPUT);
  powerX(true);
  powerY(true);
}

void setKinematicsMax(){
  stepX.setPinsInverted(INVERT_X_DIR, false, true);
  stepY.setPinsInverted(INVERT_Y_DIR, false, true);
  stepX.setMinPulseWidth(2);
  stepY.setMinPulseWidth(2);
  stepX.setMaxSpeed(MAX_FEED_MM_S * STEPS_PER_MM_X);
  stepX.setAcceleration(MAX_ACC_MM_S2 * STEPS_PER_MM_X);
  stepY.setMaxSpeed(MAX_FEED_MM_S * STEPS_PER_MM_Y);
  stepY.setAcceleration(MAX_ACC_MM_S2 * STEPS_PER_MM_Y);
}

/* ===== Setup ===== */
void setup(){
  Serial.begin(115200);
  setupPins();
  setKinematicsMax();

  EPS_X_STEPS = (long)ceilf(EPS_X_MM * STEPS_PER_MM_X);
  EPS_Y_STEPS = (long)ceilf(EPS_Y_MM * STEPS_PER_MM_Y);
  if (EPS_X_STEPS < 1) EPS_X_STEPS = 1;
  if (EPS_Y_STEPS < 1) EPS_Y_STEPS = 1;

  // latch initial states
  (void)alarmX(); (void)alarmY(); (void)pedActiveX(); (void)pedActiveY();

  if(!AX_state){ Serial.println(F("MOT_X_OK")); motX_ok_ann=true; motX_alrm_ann=false; }
  if(!AY_state){ Serial.println(F("MOT_Y_OK")); motY_ok_ann=true; motY_alrm_ann=false; }

  Serial.println(F("ok READY"));
}

/* ===== Motion helpers ===== */
void setFeed(float f_mm_min){
  float f = (f_mm_min<=0 ? 1.0f : f_mm_min/60.0f);
  stepX.setMaxSpeed(min(f*STEPS_PER_MM_X, MAX_FEED_MM_S*STEPS_PER_MM_X));
  stepY.setMaxSpeed(min(f*STEPS_PER_MM_Y, MAX_FEED_MM_S*STEPS_PER_MM_Y));
}

bool checkAlarmsAndReport(){
  bool ax = alarmX();
  bool ay = alarmY();

  if(ax && !motX_alrm_ann){ Serial.println(F("MOT_X_ALARM")); motX_alrm_ann=true; motX_ok_ann=false; }
  if(ay && !motY_alrm_ann){ Serial.println(F("MOT_Y_ALARM")); motY_alrm_ann=true; motY_ok_ann=false; }

  if(!ax && !motX_ok_ann){ Serial.println(F("MOT_X_OK")); motX_ok_ann=true; motX_alrm_ann=false; }
  if(!ay && !motY_ok_ann){ Serial.println(F("MOT_Y_OK")); motY_ok_ann=true; motY_alrm_ann=false; }

  return !(ax || ay);
}

void movePlan(float x_mm, float y_mm, float f_mm_min){
  x_mm = constrain(x_mm, X_MIN_MM, X_MAX_MM);
  y_mm = constrain(y_mm, Y_MIN_MM, Y_MAX_MM);
  setFeed(f_mm_min);
  stepX.moveTo((long)(x_mm * STEPS_PER_MM_X));
  stepY.moveTo((long)(y_mm * STEPS_PER_MM_Y));
}

/* ===== Position helpers ===== */
inline float posXmm(){ return ((float)stepX.currentPosition()) / STEPS_PER_MM_X; }
inline float posYmm(){ return ((float)stepY.currentPosition()) / STEPS_PER_MM_Y; }
inline bool atTargetXmm(float target_mm){ return fabsf(posXmm() - target_mm) <= EPS_X_MM; }
inline bool atTargetYmm(float target_mm){ return fabsf(posYmm() - target_mm) <= EPS_Y_MM; }

bool runStep(bool checkEndstops=true){
  if(checkEndstops){
    if(stepX.speed()<0 && endActive(X_MIN_PIN, X_ENDSTOP_ACTIVE_LOW)) stepX.stop();
    if(stepY.speed()<0 && endActive(Y_MIN_PIN, Y_ENDSTOP_ACTIVE_LOW)) stepY.stop();
  }
  bool rx = (stepX.distanceToGo()!=0);
  bool ry = (stepY.distanceToGo()!=0);
  stepX.run(); stepY.run();
  return rx || ry;
}

/* ===== Guarded move with PED events ===== */
/* ===== Blocking move without PED ===== */
bool guardedMoveTo(float x_mm, float y_mm, float f_mm_min){
  if(estop){ Serial.println(F("err ESTOP")); return false; }
  if(!checkAlarmsAndReport()){ return false; } // ALARM уже сообщён

  // спланировать цель с учётом лимитов и скорости
  movePlan(x_mm, y_mm, f_mm_min);

  // крутить до прихода в цель, охраняя MIN-концевики
  while(runStep(true)){ /* no-op */ }

  // успех
  return true;
}


/* ===== Homing ===== */
bool homeAxisToMin(AccelStepper& ax, uint8_t minPin, float spmm, float scanRangeMM, bool minActiveLow){
  ax.setMaxSpeed(SLOW_MM_S * spmm);
  if(endActive(minPin, minActiveLow)){
    ax.move((long)(+BACKOFF_MM * spmm));
    while(ax.distanceToGo()!=0 && endActive(minPin, minActiveLow)) ax.run();
    delay(5);
  }
  ax.setMaxSpeed(MAX_FEED_MM_S * spmm);
  long scan = (long)(scanRangeMM * spmm);
  long start = ax.currentPosition();
  ax.moveTo(start - scan);
  bool found=false;
  while(ax.distanceToGo()!=0){
    if(endActive(minPin, minActiveLow)){ ax.stop(); found=true; break; }
    ax.run();
  }
  if(!found) return false;

  ax.move((long)(+BACKOFF_MM * spmm));
  while(ax.distanceToGo()!=0) ax.run();
  ax.setMaxSpeed(SLOW_MM_S * spmm);
  ax.moveTo(ax.currentPosition() - (long)(BACKOFF_MM * spmm * 2));
  while(ax.distanceToGo()!=0){
    if(endActive(minPin, minActiveLow)){ ax.stop(); break; }
    ax.run();
  }
  ax.setCurrentPosition(0);
  return true;
}

bool homeX(){ if(!checkAlarmsAndReport()) return false; return homeAxisToMin(stepX, X_MIN_PIN, STEPS_PER_MM_X, SCAN_RANGE_X_MM, X_ENDSTOP_ACTIVE_LOW); }
bool homeY(){ if(!checkAlarmsAndReport()) return false; return homeAxisToMin(stepY, Y_MIN_PIN, STEPS_PER_MM_Y, SCAN_RANGE_Y_MM, Y_ENDSTOP_ACTIVE_LOW); }
bool homeAll(){ if(!checkAlarmsAndReport()) return false; bool fx=homeX(); bool fy=homeY(); return fx&&fy; }

/* ===== Helpers ===== */
void goZero(){ if(!checkAlarmsAndReport()) return; guardedMoveTo(0.0f, 0.0f, 1200.0f); }

void goWork(float x_mm = NAN, float y_mm = NAN, float f_mm_min = NAN){
  if(!checkAlarmsAndReport()) return;
  float X = isnan(x_mm)     ? WORK_X_MM     : x_mm;
  float Y = isnan(y_mm)     ? WORK_Y_MM     : y_mm;
  float F = isnan(f_mm_min) ? WORK_F_MM_MIN : f_mm_min;
  guardedMoveTo(X, Y, F);
}

void reportStatus(){
  float x = stepX.currentPosition()/STEPS_PER_MM_X;
  float y = stepY.currentPosition()/STEPS_PER_MM_Y;
  Serial.print("STATUS X:"); Serial.print(x,3);
  Serial.print(" Y:");      Serial.print(y,3);
  Serial.print(" X_MIN:");  Serial.print(endActive(X_MIN_PIN,X_ENDSTOP_ACTIVE_LOW)?"TRIG":"open");
  Serial.print(" Y_MIN:");  Serial.print(endActive(Y_MIN_PIN,Y_ENDSTOP_ACTIVE_LOW)?"TRIG":"open");
  Serial.print(" ESTOP:");  Serial.println(estop?"1":"0");
}

/* ===== Power-cycle reset for motor ===== */
bool resetMotor(char axis){
  if(axis=='X'){
    powerX(false); delay(RESET_OFF_MS);
    powerX(true);  delay(POWER_UP_WAIT_MS);
    if(!alarmX()){ Serial.println(F("MOT_X_OK")); motX_ok_ann=true; motX_alrm_ann=false; return true; }
    Serial.println(F("MOT_X_ALARM")); motX_ok_ann=false; motX_alrm_ann=true; return false;
  } else {
    powerY(false); delay(RESET_OFF_MS);
    powerY(true);  delay(POWER_UP_WAIT_MS);
    if(!alarmY()){ Serial.println(F("MOT_Y_OK")); motY_ok_ann=true; motY_alrm_ann=false; return true; }
    Serial.println(F("MOT_Y_ALARM")); motY_ok_ann=false; motY_alrm_ann=true; return false;
  }
}

/* ===== Commands =====
   РќРѕРІС‹Рµ:
     MOT_X_RESET / MOT_Y_RESET
   Р‘Р»РѕРєРёСЂРѕРІРєР° Р»СЋР±РѕРіРѕ РґРІРёР¶РµРЅРёСЏ/С…РѕРјРёРЅРіР° РїСЂРё Р°РєС‚РёРІРЅРѕРј ALARM:
     РїСЂРё РїРѕРїС‹С‚РєРµ вЂ” РїРµС‡Р°С‚Р°РµС‚СЃСЏ MOT_X_ALARM / MOT_Y_ALARM (РѕРґРёРЅ СЂР°Р·)
*/
void handleLine(String s){
  s.trim(); if(!s.length()) return;

  // === NEW: прямой статус по запросу, без триггера одноразовой отчетности ===
  if(s=="MOT_X_STATUS"){ Serial.println(alarmX() ? F("MOT_X_ALARM") : F("MOT_X_OK")); return; }
  if(s=="MOT_Y_STATUS"){ Serial.println(alarmY() ? F("MOT_Y_ALARM") : F("MOT_Y_OK")); return; }

  // Актуализируем ALM/OK одноразовые сообщения (пусть остается как было)
  checkAlarmsAndReport();

  if(s=="MOT_X_RESET"){ resetMotor('X'); return; }
  if(s=="MOT_Y_RESET"){ resetMotor('Y'); return; }

  if(s=="PING"){ Serial.println("PONG"); return; }
  if(s=="M114"){ reportStatus(); Serial.println("ok"); return; }
  if(s=="M119"){
    Serial.print("X_MIN:"); Serial.print(endActive(X_MIN_PIN,X_ENDSTOP_ACTIVE_LOW)?"TRIGGERED":"open"); Serial.print(" ");
    Serial.print("Y_MIN:"); Serial.println(endActive(Y_MIN_PIN,Y_ENDSTOP_ACTIVE_LOW)?"TRIGGERED":"open");
    Serial.println("ok"); return;
  }
  if(s=="M112"){ estop=true; stepX.stop(); stepY.stop(); motorsEnable(false); Serial.println("ok ESTOP"); return; }
  if(s=="M999"){ estop=false; motorsEnable(true); Serial.println("ok CLEAR"); return; }

  if(s=="G28"){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    Serial.println(homeAll() ? "ok IN_HOME_POS" : "err HOME_NOT_FOUND"); return;
  }
  if(s=="G28 X"){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    Serial.println(homeX() ? "ok IN_X_HOME_POS" : "err HOME_X_NOT_FOUND"); return;
  }
  if(s=="G28 Y"){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    Serial.println(homeY() ? "ok IN_Y_HOME_POS" : "err HOME_Y_NOT_FOUND"); return;
  }
  if(s=="CAL"){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    if(homeAll()){ goZero(); Serial.println("ok"); } else { Serial.println("err HOME_NOT_FOUND"); }
    return;
  }
  if(s=="ZERO"){ if(estop){ Serial.println("err ESTOP"); return; } goZero(); Serial.println("ok"); return; }

  if(s.startsWith("SET LIM ")){
    float xmx=X_MAX_MM, ymx=Y_MAX_MM;
    int i=8; while(i<s.length()){
      int j=s.indexOf(' ', i); if(j<0) j=s.length();
      String t=s.substring(i,j);
      if(t.startsWith("X")) xmx=t.substring(1).toFloat();
      else if(t.startsWith("Y")) ymx=t.substring(1).toFloat();
      i=j+1;
    }
    X_MIN_MM=0.0f; X_MAX_MM=xmx; Y_MIN_MM=0.0f; Y_MAX_MM=ymx;
    Serial.println("ok"); return;
  }

  if(s.startsWith("SET STEPS ")){
    float xs=STEPS_PER_MM_X, ys=STEPS_PER_MM_Y;
    int i=10; while(i<s.length()){
      int j=s.indexOf(' ', i); if(j<0) j=s.length();
      String t=s.substring(i,j);
      if     (t.startsWith("X")) xs=t.substring(1).toFloat();
      else if(t.startsWith("Y")) ys=t.substring(1).toFloat();
      i=j+1;
    }
    STEPS_PER_MM_X=xs; STEPS_PER_MM_Y=ys; setKinematicsMax();
    // ← ВСТАВИТЬ ЗДЕСЬ:
    EPS_X_STEPS = (long)ceilf(EPS_X_MM * STEPS_PER_MM_X);
    EPS_Y_STEPS = (long)ceilf(EPS_Y_MM * STEPS_PER_MM_Y);
    if (EPS_X_STEPS < 1) EPS_X_STEPS = 1;
    if (EPS_Y_STEPS < 1) EPS_Y_STEPS = 1;
    Serial.println("ok"); return;
  }

  if(s.startsWith("WORK")){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    float x = NAN, y = NAN, f = NAN;
    if(s.length() > 4){
      int i=5;
      while(i < s.length()){
        int j = s.indexOf(' ', i); if(j < 0) j = s.length();
        String t = s.substring(i, j);
        if     (t.startsWith("X")) x = t.substring(1).toFloat();
        else if(t.startsWith("Y")) y = t.substring(1).toFloat();
        else if(t.startsWith("F")) f = t.substring(1).toFloat();
        i = j + 1;
      }
    }
    float X = isnan(x) ? WORK_X_MM : x;
    float Y = isnan(y) ? WORK_Y_MM : y;
    float F = isnan(f) ? WORK_F_MM_MIN : f;
    guardedMoveTo(X, Y, F);
    Serial.println("ok");
    return;
  }

  if(s.startsWith("SET WORK ")){
    float x = WORK_X_MM, y = WORK_Y_MM, f = WORK_F_MM_MIN;
    int i=9;
    while(i < s.length()){
      int j = s.indexOf(' ', i); if(j < 0) j = s.length();
      String t = s.substring(i, j);
      if     (t.startsWith("X")) x = t.substring(1).toFloat();
      else if(t.startsWith("Y")) y = t.substring(1).toFloat();
      else if(t.startsWith("F")) f = t.substring(1).toFloat();
      i = j + 1;
    }
    WORK_X_MM = x; WORK_Y_MM = y; WORK_F_MM_MIN = f;
    Serial.println("ok");
    return;
  }

  // Diagnostic jogs (DX/DY)
  if(s.startsWith("DX ")){
    float d=0, f=600; int i=3;
    while(i<s.length()){ int j=s.indexOf(' ',i); if(j<0) j=s.length(); String t=s.substring(i,j);
      if(t.startsWith("+")||t.startsWith("-")) d=t.toFloat();
      else if(t.startsWith("F")) f=t.substring(1).toFloat();
      i=j+1;}
    if(!checkAlarmsAndReport()) return;

    setFeed(f);
    stepX.move(stepX.currentPosition() + (long)(d*STEPS_PER_MM_X));
    while(runStep(true)){}        // ← просто доехать
    Serial.println("ok"); return;
  }

  if(s.startsWith("DY ")){
    float d=0, f=600; int i=3;
    while(i<s.length()){ int j=s.indexOf(' ',i); if(j<0) j=s.length(); String t=s.substring(i,j);
      if(t.startsWith("+")||t.startsWith("-")) d=t.toFloat();
      else if(t.startsWith("F")) f=t.substring(1).toFloat();
      i=j+1;}
    if(!checkAlarmsAndReport()) return;

    setFeed(f);
    stepY.move(stepY.currentPosition() + (long)(d*STEPS_PER_MM_Y));
    while(runStep(true)){}        // ← просто доехать
    Serial.println("ok"); return;
  }

  if(s.startsWith("GF ")){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    float x=NAN, y=NAN, f=1200;
    int i=3; while(i<s.length()){
      int j=s.indexOf(' ', i); if(j<0) j=s.length();
      String t=s.substring(i,j);
      if      (t.startsWith("X")) x=t.substring(1).toFloat();
      else if (t.startsWith("Y")) y=t.substring(1).toFloat();
      else if (t.startsWith("F")) f=t.substring(1).toFloat();
      i=j+1;
    }
    if(isnan(x) || isnan(y)){ Serial.println("err BAD_ARGS"); return; }
    movePlan(x,y,f);
    while(runStep(true)) { /* ездим, MIN-концевики охраняем, PED игнорим */ }
    Serial.println("ok"); return;
  }


  if(s.startsWith("G ")){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(!checkAlarmsAndReport()) return;
    float x=NAN,y=NAN,f=1200;
    int i=2; while(i<s.length()){
      int j=s.indexOf(' ', i); if(j<0) j=s.length();
      String t=s.substring(i,j);
      if     (t.startsWith("X")) x=t.substring(1).toFloat();
      else if(t.startsWith("Y")) y=t.substring(1).toFloat();
      else if(t.startsWith("F")) f=t.substring(1).toFloat();
      i=j+1;
    }
    if(isnan(x)||isnan(y)){ Serial.println("err BAD_ARGS"); return; }
    guardedMoveTo(x,y,f);
    Serial.println("ok"); return;
  }

  Serial.println("err UNKNOWN");
}

void loop(){
  // РЎР»РµРґРёРј Р·Р° ALM РґР°Р¶Рµ РІРЅРµ РєРѕРјР°РЅРґ (С‡С‚РѕР±С‹ РѕС‚РґР°С‚СЊ MOT_*_OK/ALARM РѕРґРёРЅ СЂР°Р·)
  checkAlarmsAndReport();

  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){ handleLine(ibuf); ibuf=""; }
    else { ibuf+=c; if(ibuf.length()>160) ibuf=""; }
  }
}