#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

// ESP32 Furby unified motion-controller prototype, no audio yet.
// Preserves the known-good TALK routine's target positions but replaces its
// motor loop with a single shared positioning service. Test with shell off.
constexpr uint8_t BUTTON=14, FWD_PIN=16, REV_PIN=17, ENABLE_PIN=32;
constexpr uint8_t HOME_PIN=13, ENCODER_PIN=33;
constexpr int CYCLE=208, CLOSED=140, OPEN=175;
constexpr uint32_t HOME_TIMEOUT=6000, MOVE_TIMEOUT=4500, SETTLE_TIMEOUT=2000;
constexpr uint32_t MIN_SETTLE=300, QUIET=110;
constexpr uint32_t BUTTON_DEBOUNCE=50;
constexpr int HOME_WINDOW=10, MAX_HOME_SEARCH_TICKS=440;
constexpr int TARGET_TOL=5, MAX_POSITION_ERROR=12, MAX_TRIMS=10;
constexpr int MIN_CONTINUOUS=24;

// IMPORTANT: encoder is single-channel. Sign is the COMMANDED direction.
// A tick during motor coast is attributed to that same direction.
enum Dir:int8_t { STOP=0, FORWARD=1, REVERSE=-1 };
enum Phase:uint8_t { IDLE, SEEK_HOME, POWERED, SETTLING, FAULT };
enum Job:uint8_t { NO_JOB, HOME_JOB, POSITION_JOB };
Phase phase=IDLE; Job job=NO_JOB; Dir direction=STOP;
bool synced=false, demo=false, talk=false, buttonRaw=HIGH, buttonStable=HIGH;
bool lastEncoder=HIGH, lastHome=HIGH, homeArmed=false;
int32_t position=0; // UNWRAPPED encoder coordinate, zero at a verified HOME edge
int32_t target=0, cutoff=0, homeSeekTicks=0;
int32_t poweredTicks=0, coastTicks=0;
int coastFwd=19, coastRev=16, trims=0;
bool shortPulse=false; uint16_t nextPauseMs=0;
uint32_t phaseStarted=0, motorOffAt=0, lastTickAt=0;
uint32_t buttonChanged=0;
int32_t lastHomePosition=INT32_MIN;
uint32_t pauseUntil=0; uint8_t demoStep=0, talkStep=0;
char commandBuf[32]; uint8_t commandLen=0;

struct Pose { int tick; uint16_t pause; const char* name; };
const Pose poses[]={
  {30,700,"CURIOUS: ears raised"}, {70,850,"CONTEMPLATIVE: eyes closed"},
  {105,750,"SAD: ears lowered"}, {140,650,"ATTENTIVE: eyes open"},
  {173,950,"SURPRISED: mouth open"}, {202,700,"QUIET: mouth closing"}
};
constexpr uint8_t POSE_COUNT=sizeof(poses)/sizeof(poses[0]);
int wrap(int32_t p){int v=p%CYCLE;return v<0?v+CYCLE:v;}
int shortest(int from,int to){int d=wrap(to-from);return d>104?d-208:d;}
void motorOff(){digitalWrite(FWD_PIN,LOW);digitalWrite(REV_PIN,LOW);}
void motorOn(Dir d){motorOff();if(d==FORWARD)digitalWrite(FWD_PIN,HIGH);else if(d==REVERSE)digitalWrite(REV_PIN,HIGH);}
void fault(const char* why){motorOff();phase=FAULT;job=NO_JOB;direction=STOP;synced=false;demo=talk=false;nextPauseMs=0;
  Serial.print("FAULT: ");Serial.println(why);Serial.println("Inspect Furby; send H to re-home or X to stop.");}
void status(){Serial.print("STATUS | synced ");Serial.print(synced?"yes":"no");
  Serial.print(" | pos ");Serial.print(wrap(position));Serial.print(" | unwrapped ");Serial.print(position);
  Serial.print(" | phase ");Serial.print((int)phase);Serial.print(" | coast F/R ");
  Serial.print(coastFwd);Serial.print('/');Serial.println(coastRev);}
void beginSettle(){motorOff();motorOffAt=millis();lastTickAt=motorOffAt;phase=SETTLING;coastTicks=0;}
void startHome(){
  motorOff();demo=talk=false;nextPauseMs=0;synced=false;job=HOME_JOB;phase=SEEK_HOME;direction=FORWARD;
  homeSeekTicks=0;position=0;lastEncoder=digitalRead(ENCODER_PIN);
  lastHome=digitalRead(HOME_PIN);homeArmed=(lastHome==HIGH);
  lastHomePosition=INT32_MIN;
  phaseStarted=millis();lastTickAt=phaseStarted;
  motorOn(FORWARD);Serial.println("HOMING: waiting for HOME HIGH-to-LOW edge...");
}
// The HOME active region can be much shorter than 12 ms. Detect the raw
// HIGH-to-LOW transition; do not require the signal to remain LOW.
// Require a HIGH observation to arm each edge. During normal motion, a
// position-based revolution lockout prevents repeated triggers in one region.
void readHome(){
  bool v=digitalRead(HOME_PIN);
  if(v==HIGH)homeArmed=true;
  bool falling=(lastHome==HIGH && v==LOW);
  lastHome=v;
  if(!falling || !homeArmed)return;
  homeArmed=false;
  if(phase==SEEK_HOME){
    position=0;synced=true;lastHomePosition=0;
    Serial.println("HOME edge found: reference zero; waiting for coast...");
    beginSettle();return;
  }
  if(!synced || (phase!=POWERED && phase!=SETTLING))return;
  // HOME should appear once per 208-tick cam revolution. Ignore extra
  // edges in the same HOME region (as observed in earlier mapping logs).
  if(lastHomePosition!=INT32_MIN && abs(position-lastHomePosition)<CYCLE/2){
    Serial.println("HOME chatter ignored (same revolution)");return;
  }
  int p=wrap(position);int err=p<=CYCLE/2?p:p-CYCLE;
  Serial.print("HOME edge | predicted tick ");Serial.print(p);
  Serial.print(" | discrepancy ");Serial.println(err);
  if(abs(err)>HOME_WINDOW){
    // Do not silently snap the counter to a suspect HOME transition.
    fault("HOME disagrees with encoder; position uncertain");return;
  }
  position-=err;
  lastHomePosition=position;
  Serial.println("HOME verified; encoder aligned to reference");
}
void readEncoder(){
  bool v=digitalRead(ENCODER_PIN);
  if(lastEncoder==HIGH && v==LOW){
    lastTickAt=millis();
    if(phase==SEEK_HOME){++homeSeekTicks;}
    else if(synced && direction!=STOP && (phase==POWERED||phase==SETTLING)){
      position+=(int)direction;
      if(phase==POWERED)++poweredTicks;else ++coastTicks;
    }
  }
  lastEncoder=v;
}
void startPower(Dir d,int32_t dest,bool pulse=false){
  shortPulse=pulse;
  direction=d;cutoff=dest;poweredTicks=0;coastTicks=0;
  phaseStarted=millis();phase=POWERED;motorOn(d);
}
void beginMove(int dest,bool forceForward=false){
  if(!synced || phase!=IDLE || job!=NO_JOB){Serial.println("Not ready; send H or wait.");return;}
  int delta=forceForward?wrap(dest-wrap(position)):shortest(wrap(position),dest);
  if(forceForward && delta==0 && dest!=wrap(position))delta=CYCLE;
  target=position+delta;job=POSITION_JOB;trims=0;
  Serial.print("MOVE | from ");Serial.print(wrap(position));Serial.print(" to ");Serial.print(dest);
  Serial.print(" | delta ");Serial.println(delta);
  if(abs(delta)<=TARGET_TOL){job=NO_JOB;Serial.println("Already within tolerance.");return;}
  Dir d=delta>0?FORWARD:REVERSE;
  int estimate=d==FORWARD?coastFwd:coastRev;
  // Long move: power continuously, cut early to anticipate coast.
  // Short move: brief pulse; subsequent pulses only after complete settling.
  if(abs(delta)>=MIN_CONTINUOUS && abs(delta)>estimate+3){
    startPower(d,target-(int)d*estimate);
  }else{
    startPower(d,position+(int)d*1,true);
  }
}
void startDemo(){if(!synced||phase!=IDLE||job!=NO_JOB){Serial.println("Home and wait first.");return;}
  demo=true;talk=false;demoStep=0;pauseUntil=0;nextPauseMs=0;Serial.println("EXPRESSION SEQUENCE START");}
void startTalk(){if(!synced||phase!=IDLE||job!=NO_JOB){Serial.println("Home and wait first.");return;}
  demo=false;talk=true;talkStep=0;pauseUntil=0;nextPauseMs=0;Serial.println("TALK TEST: 6 cycles");}
void finishMove(){
  int error=(int)(target-position);int actual=wrap(position);
  Serial.print("SETTLED | tick ");Serial.print(actual);Serial.print(" | target ");
  Serial.print(wrap(target));Serial.print(" | error ");Serial.print(error);
  Serial.print(" | coast ");Serial.println(coastTicks);
  if(abs(error)>MAX_POSITION_ERROR){fault("position error exceeds safety limit");return;}
  if(abs(error)<=TARGET_TOL){job=NO_JOB;direction=STOP;pauseUntil=millis()+nextPauseMs;nextPauseMs=0;return;}
  if(++trims>MAX_TRIMS){fault("too many position corrections");return;}
  // Small pulse, then settle again; do not use long-run coast estimate.
  Dir d=error>0?FORWARD:REVERSE;
  Serial.print("TRIM ");Serial.println(d==FORWARD?"forward":"reverse");
  startPower(d,position+(int)d,true);
}
void sequence(){
  if(phase!=IDLE||!synced||job!=NO_JOB||millis()<pauseUntil)return;
  if(demo){
    if(demoStep>=POSE_COUNT){demo=false;Serial.println("EXPRESSION SEQUENCE COMPLETE");return;}
    const Pose &p=poses[demoStep++];
    Serial.print("EXPRESSION | ");Serial.println(p.name);
    nextPauseMs=p.pause;
    beginMove(p.tick,true);return;
  }
  if(talk){
    if(talkStep>=12){talk=false;Serial.println("TALK TEST COMPLETE");return;}
    int dest=(talkStep++%2==0)?OPEN:CLOSED;
    nextPauseMs=100;
    beginMove(dest);return;
  }
}
void handle(const char*s){
  if(!*s)return;
  if(s[1]=='\0')switch(toupper((unsigned char)s[0])){
    case 'H':startHome();return;
    case 'S':status();return;
    case 'X':fault("manual stop");return;
    case 'D':startDemo();return;
    case 'T':startTalk();return;
    case '?':Serial.println("H home | D expressions | T 6 talk cycles | S status | X stop | 0..207 position");return;
  }
  char*end=nullptr;long p=strtol(s,&end,10);
  if(end!=s&&*end=='\0'&&p>=0&&p<CYCLE){demo=talk=false;beginMove((int)p);return;}
  Serial.println("Unknown command; send ?");
}
void serialTask(){while(Serial.available()){
  char c=(char)Serial.read();if(c=='\n'||c=='\r'){
    if(commandLen){commandBuf[commandLen]='\0';handle(commandBuf);commandLen=0;}
  }else if(commandLen<sizeof(commandBuf)-1)commandBuf[commandLen++]=c;
}}
void buttonTask(){bool b=digitalRead(BUTTON);if(b!=buttonRaw){buttonRaw=b;buttonChanged=millis();}
  if(millis()-buttonChanged>=BUTTON_DEBOUNCE&&b!=buttonStable){buttonStable=b;
    if(b==LOW)startDemo();}}
void setup(){
  Serial.begin(115200);
  digitalWrite(FWD_PIN,LOW);digitalWrite(REV_PIN,LOW);
  pinMode(FWD_PIN,OUTPUT);pinMode(REV_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,HIGH);pinMode(ENABLE_PIN,OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);pinMode(HOME_PIN,INPUT_PULLUP);pinMode(ENCODER_PIN,INPUT_PULLUP);
  motorOff();lastEncoder=digitalRead(ENCODER_PIN);lastHome=digitalRead(HOME_PIN);
  buttonRaw=buttonStable=digitalRead(BUTTON);
  Serial.println("FURBY UNIFIED MOTION CONTROLLER - HOME EDGE FIX v2");
  Serial.println("H home | D expressions | T talk | S status | X stop | 0..207 target");
}
void loop(){
  readEncoder();readHome();serialTask();buttonTask();uint32_t now=millis();
  if(phase==SEEK_HOME){
    if(now-phaseStarted>HOME_TIMEOUT||homeSeekTicks>MAX_HOME_SEARCH_TICKS)
      fault("HOME search timeout");
    return;
  }
  if(phase==POWERED){
    if(now-phaseStarted>MOVE_TIMEOUT){fault("motor run timeout");return;}
    if((direction==FORWARD&&position>=cutoff)||(direction==REVERSE&&position<=cutoff)){
      beginSettle();return;
    }
    // For a short trim the motor is allowed at most 20 ms powered.
    if(shortPulse && now-phaseStarted>=18){beginSettle();return;}
    return;
  }
  if(phase==SETTLING){
    if(now-motorOffAt>SETTLE_TIMEOUT){fault("settling timeout");return;}
    if(now-motorOffAt<MIN_SETTLE||now-lastTickAt<QUIET)return;
    phase=IDLE;
    if(job==HOME_JOB){job=NO_JOB;direction=STOP;
      Serial.print("HOME COMPLETE | settled tick ");Serial.println(wrap(position));return;}
    if(job==POSITION_JOB){
      // Update coast estimate only after a genuine long powered run.
      if(poweredTicks>=6 && coastTicks>=5 && coastTicks<=35){
        int &estimate=(direction==FORWARD?coastFwd:coastRev);
        estimate=constrain((estimate*3+coastTicks+2)/4,8,30);
      }
      finishMove();return;
    }
    direction=STOP;return;
  }
  sequence();
}
