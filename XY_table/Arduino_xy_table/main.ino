#include <AccelStepper.h>

/* ===== RAMPS 1.4 pins (Mega2560) ===== */
#define X_STEP 54
#define X_DIR  55
#define X_EN   38
#define Y_STEP 60
#define Y_DIR  61
#define Y_EN   56

// Endstops (MIN only)
#define X_MIN_PIN 3     // X-MIN
#define Y_MIN_PIN 14    // Y-MIN

/* ===== CONFIG: mechanics & logic ===== */
// TR8x8, 800 imp/rev  => 800/8 = 100 steps/mm
float STEPS_PER_MM_X = 50.0f;
float STEPS_PER_MM_Y = 50.0f;

// мягкие стартовые параметры
float MAX_FEED_MM_S  = 600.0f;   // мм/с
float MAX_ACC_MM_S2  = 60000.0f;  // мм/с^2

// Рабочие лимиты (мм): 0…MAX (ноль на MIN)
float X_MIN_MM = 0.0f, X_MAX_MM = 60.0f;
float Y_MIN_MM = 0.0f, Y_MAX_MM = 160.0f;

// Homing scan & finesse
float SCAN_RANGE_X_MM = 65.0f;
float SCAN_RANGE_Y_MM = 165.0f;
float BACKOFF_MM      = 3.0f;
float SLOW_MM_S       = 8.0f;

// Direction inversion (перевернуть, если ось едет «не туда»)
bool INVERT_X_DIR = true;
bool INVERT_Y_DIR = true;

// Endstop electrical logic: NC → active LOW (true),  NO → active HIGH (false)
bool X_ENDSTOP_ACTIVE_LOW = false;  // NC по умолчанию
bool Y_ENDSTOP_ACTIVE_LOW = false;  // NC по умолчанию

/* ===================================== */

AccelStepper stepX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP, Y_DIR);

String ibuf;
bool estop=false;

inline bool endActive(uint8_t pin, bool activeLow){
  int v = digitalRead(pin);
  return activeLow ? (v==LOW) : (v==HIGH);
}

void motorsEnable(bool en){
  digitalWrite(X_EN, en?LOW:HIGH); // RAMPS: LOW=enable
  digitalWrite(Y_EN, en?LOW:HIGH);
}

void setupPins(){
  pinMode(X_EN, OUTPUT); pinMode(Y_EN, OUTPUT);
  motorsEnable(true);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
}

void setKinematicsMax(){
  stepX.setPinsInverted(INVERT_X_DIR, false, true); // dir, step, enable
  stepY.setPinsInverted(INVERT_Y_DIR, false, true);
  stepX.setMinPulseWidth(2);  // μs
  stepY.setMinPulseWidth(2);
  stepX.setMaxSpeed(MAX_FEED_MM_S * STEPS_PER_MM_X);
  stepX.setAcceleration(MAX_ACC_MM_S2 * STEPS_PER_MM_X);
  stepY.setMaxSpeed(MAX_FEED_MM_S * STEPS_PER_MM_Y);
  stepY.setAcceleration(MAX_ACC_MM_S2 * STEPS_PER_MM_Y);
}

void setup(){
  Serial.begin(115200);
  setupPins();
  setKinematicsMax();
  Serial.println("ok READY");
}

/* ===== motion ===== */
void setFeed(float f_mm_min){
  float f = (f_mm_min<=0 ? 1.0f : f_mm_min/60.0f);
  stepX.setMaxSpeed(min(f*STEPS_PER_MM_X, MAX_FEED_MM_S*STEPS_PER_MM_X));
  stepY.setMaxSpeed(min(f*STEPS_PER_MM_Y, MAX_FEED_MM_S*STEPS_PER_MM_Y));
}
void movePlan(float x_mm, float y_mm, float f_mm_min){
  x_mm = constrain(x_mm, X_MIN_MM, X_MAX_MM);
  y_mm = constrain(y_mm, Y_MIN_MM, Y_MAX_MM);
  setFeed(f_mm_min);
  stepX.moveTo((long)(x_mm * STEPS_PER_MM_X));
  stepY.moveTo((long)(y_mm * STEPS_PER_MM_Y));
}
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

/* ===== homing to MIN (пер-ось) ===== */
bool homeAxisToMin(AccelStepper& ax, uint8_t minPin, float spmm, float scanRangeMM, bool minActiveLow){
  // если стоим на концевике — отъедем
  ax.setMaxSpeed(SLOW_MM_S * spmm);
  if(endActive(minPin, minActiveLow)){
    ax.move((long)(+BACKOFF_MM * spmm));
    while(ax.distanceToGo()!=0 && endActive(minPin, minActiveLow)) ax.run();
    delay(5);
  }
  // быстрый поиск в "минус"
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

  // отъезд + медленный точный заход
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

bool homeX(){ return homeAxisToMin(stepX, X_MIN_PIN, STEPS_PER_MM_X, SCAN_RANGE_X_MM, X_ENDSTOP_ACTIVE_LOW); }
bool homeY(){ return homeAxisToMin(stepY, Y_MIN_PIN, STEPS_PER_MM_Y, SCAN_RANGE_Y_MM, Y_ENDSTOP_ACTIVE_LOW); }

bool homeAll(){
  bool fx = homeX();
  bool fy = homeY();
  return fx && fy;
}

/* ===== utils ===== */
void goZero(){ movePlan(0.0f, 0.0f, 1200.0f); while(runStep()){} }

/* ===== reports ===== */
void reportStatus(){
  float x = stepX.currentPosition()/STEPS_PER_MM_X;
  float y = stepY.currentPosition()/STEPS_PER_MM_Y;
  Serial.print("STATUS X:"); Serial.print(x,3);
  Serial.print(" Y:");      Serial.print(y,3);
  Serial.print(" X_MIN:");  Serial.print(endActive(X_MIN_PIN,X_ENDSTOP_ACTIVE_LOW)?"TRIG":"open");
  Serial.print(" Y_MIN:");  Serial.print(endActive(Y_MIN_PIN,Y_ENDSTOP_ACTIVE_LOW)?"TRIG":"open");
  Serial.print(" ESTOP:");  Serial.println(estop?"1":"0");
}

/* ===== protocol ===== */
/*
Команды:
  PING
  M114 / M119
  M112 / M999
  G28         -> хоум X и Y
  G28 X       -> хоум только X
  G28 Y       -> хоум только Y
  CAL         -> хоум обеих + ZERO
  ZERO        -> в (0,0)
  G X.. Y.. F..
  SET LIM X300 Y300
  SET STEPS X100 Y100
  DX +10 F600 -> сдвиг X на +10 мм (диагностика, без софт-лимитов)
  DY -5 F600  -> сдвиг Y на -5 мм (диагностика, без софт-лимитов)
*/
void handleLine(String s){
  s.trim(); if(!s.length()) return;

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
    Serial.println(homeAll() ? "ok" : "err HOME_NOT_FOUND"); return;
  }
  if(s=="G28 X"){
    if(estop){ Serial.println("err ESTOP"); return; }
    Serial.println(homeX() ? "ok" : "err HOME_X_NOT_FOUND"); return;
  }
  if(s=="G28 Y"){
    if(estop){ Serial.println("err ESTOP"); return; }
    Serial.println(homeY() ? "ok" : "err HOME_Y_NOT_FOUND"); return;
  }
  if(s=="CAL"){
    if(estop){ Serial.println("err ESTOP"); return; }
    if(homeAll()){ goZero(); Serial.println("ok"); } else { Serial.println("err HOME_NOT_FOUND"); }
    return;
  }
  if(s=="ZERO"){
    if(estop){ Serial.println("err ESTOP"); return; }
    goZero(); Serial.println("ok"); return;
  }

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
    Serial.println("ok"); return;
  }

  // Диагностические сдвиги (без софт-лимитов; с проверкой концевиков)
  if(s.startsWith("DX ")){
    float d=0, f=600; int i=3; 
    while(i<s.length()){ int j=s.indexOf(' ',i); if(j<0) j=s.length(); String t=s.substring(i,j);
      if(t.startsWith("+")||t.startsWith("-")) d=t.toFloat();
      else if(t.startsWith("F")) f=t.substring(1).toFloat();
      i=j+1;}
    setFeed(f); stepX.move(stepX.currentPosition() + (long)(d*STEPS_PER_MM_X));
    while(runStep(true)){} Serial.println("ok"); return;
  }
  if(s.startsWith("DY ")){
    float d=0, f=600; int i=3; 
    while(i<s.length()){ int j=s.indexOf(' ',i); if(j<0) j=s.length(); String t=s.substring(i,j);
      if(t.startsWith("+")||t.startsWith("-")) d=t.toFloat();
      else if(t.startsWith("F")) f=t.substring(1).toFloat();
      i=j+1;}
    setFeed(f); stepY.move(stepY.currentPosition() + (long)(d*STEPS_PER_MM_Y));
    while(runStep(true)){} Serial.println("ok"); return;
  }

  if(s.startsWith("G ")){
    if(estop){ Serial.println("err ESTOP"); return; }
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
    movePlan(x,y,f);
    while(runStep()){}
    Serial.println("ok"); return;
  }

  Serial.println("err UNKNOWN");
}

void loop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){ handleLine(ibuf); ibuf=""; }
    else { ibuf+=c; if(ibuf.length()>160) ibuf=""; }
  }
}
