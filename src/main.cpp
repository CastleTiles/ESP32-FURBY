#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>

// Furby adaptive mouth controller - ESP32 / PlatformIO Arduino
constexpr uint8_t BUTTON_PIN=14, MOTOR_FWD_PIN=16, MOTOR_REV_PIN=17;
constexpr uint8_t POWER_DOWN_PIN=32, CAM_HOME_PIN=13, GEAR_ROTATION_PIN=33;
constexpr int REV_TICKS=208, CLOSED=140, OPEN=175;
constexpr int BASE_OPEN_CUTOFF=154, BASE_CLOSE_CUTOFF=160;
constexpr int MAX_ADJUST=5, OPEN_TOL=8, CLOSE_TOL=6;
constexpr uint32_t NORMAL_QUIET_MS=500, FAST_QUIET_MS=100;
constexpr uint32_t MIN_FAST_COAST_MS=300, MAX_SETTLE_MS=2000;
constexpr uint32_t HOME_TIMEOUT_MS=5000, MOVE_TIMEOUT_MS=15000, MOTION_TIMEOUT_MS=1500;
constexpr int MAX_MOVE_PULSES=80, MOVE_TOL=3, TALK_CYCLES=20;
constexpr uint32_t LARGE_PULSE_MS=50, SMALL_PULSE_MS=20;
constexpr uint32_t MIN_COAST_TICKS=8;

enum Direction:int8_t { STOP=0, FWD=1, REV=-1 };
enum Mode:uint8_t { IDLE, HOMING, MOVING, PULSING, OPENING, CLOSING, SETTLING };
enum Settle:uint8_t { NONE, HOME, MOVE, OPEN_DONE, CLOSE_DONE };
enum Next:uint8_t { NO_NEXT, NEXT_OPEN, NEXT_CLOSE };
Mode mode=IDLE;
Settle settling=NONE;
Next nextTalk=NO_NEXT;
Direction direction=STOP;
int pos=0, target=0, cutoffPos=0;
int openAdjust=0, closeAdjust=0;
bool synced=false, talking=false;
bool lastGear=HIGH, lastHome=HIGH, homeArmed=false;
uint32_t totalTicks=0, poweredTicks=0, coastTicks=0;
uint32_t operationStart=0, pulseStart=0, pulseLength=0, motorOffAt=0;
uint32_t lastTickAt=0, lastCoastAt=0, motionStart=0, motionCutoffAt=0, talkStart=0;
int movePulses=0, talkCycles=0;
char command[24]; uint8_t commandLength=0;

int wrap(int p){ p%=REV_TICKS; return p<0?p+REV_TICKS:p; }
int distance(int from,int to){ int d=wrap(to-from); return d>REV_TICKS/2?d-REV_TICKS:d; }
void motorOff(){digitalWrite(MOTOR_FWD_PIN,LOW);digitalWrite(MOTOR_REV_PIN,LOW);}
void motorOn(Direction d){motorOff(); if(d==FWD)digitalWrite(MOTOR_FWD_PIN,HIGH);else if(d==REV)digitalWrite(MOTOR_REV_PIN,HIGH);}
void resetMotion(){mode=IDLE;settling=NONE;direction=STOP;nextTalk=NO_NEXT;talking=false;}
void abortMotion(const char* why){motorOff();resetMotion();synced=false;Serial.print("ABORT: ");Serial.println(why);Serial.println("Position uncertain; use H to re-home.");}
void encoder(){
  bool v=digitalRead(GEAR_ROTATION_PIN);
  if(lastGear==HIGH && v==LOW){
    uint32_t t=millis(); ++totalTicks;
    if(mode==PULSING||mode==OPENING||mode==CLOSING)++poweredTicks;
    else if(mode==SETTLING){++coastTicks;lastCoastAt=t;}
    if(synced && direction!=STOP)pos=wrap(pos+(int)direction);
    lastTickAt=t;
  }
  lastGear=v;
}
void beginSettle(Settle reason){
  motorOff();motorOffAt=millis();lastTickAt=motorOffAt;lastCoastAt=motorOffAt;
  settling=reason;mode=SETTLING;
}
void homeSensor(){
  bool v=digitalRead(CAM_HOME_PIN);
  if(v==HIGH)homeArmed=true;
  if(lastHome==HIGH && v==LOW && homeArmed){
    homeArmed=false;
    if(mode==HOMING){
      pos=0;synced=true;direction=FWD;
      Serial.println("HOME detected.");beginSettle(HOME);
    }
  }
  lastHome=v;
}
void startHome(){
  if(mode!=IDLE||talking){Serial.println("Busy.");return;}
  synced=false;direction=FWD;operationStart=millis();mode=HOMING;
  Serial.println("Searching for HOME...");motorOn(FWD);
}
void startMove(int p){
  if(mode!=IDLE||talking){Serial.println("Busy.");return;}
  if(!synced){Serial.println("Use H first.");return;}
  target=wrap(p);movePulses=0;operationStart=millis();mode=MOVING;
  Serial.print("Moving to ");Serial.println(target);
}
void continueMove(){
  if(mode!=MOVING)return;
  int d=distance(pos,target);
  if(abs(d)<=MOVE_TOL){mode=IDLE;direction=STOP;Serial.print("TARGET REACHED: ");Serial.println(pos);return;}
  if(++movePulses>MAX_MOVE_PULSES||millis()-operationStart>MOVE_TIMEOUT_MS){abortMotion("positioning timeout");return;}
  direction=d>0?FWD:REV;
  pulseLength=abs(d)>20?LARGE_PULSE_MS:SMALL_PULSE_MS;
  poweredTicks=coastTicks=0;pulseStart=millis();mode=PULSING;
  motorOn(direction);
  Serial.print("Pulse ");Serial.print(direction==FWD?"FWD ":"REV ");Serial.print(pulseLength);Serial.println(" ms");
}
int openCutoff(){return BASE_OPEN_CUTOFF+openAdjust;}
int closeCutoff(){return BASE_CLOSE_CUTOFF+closeAdjust;}
bool startMouth(bool opening){
  if(mode!=IDLE||!synced)return false;
  const int reference=opening?CLOSED:OPEN;
  const int allowed=opening?6:9;
  if(abs(distance(pos,reference))>allowed)return false;
  poweredTicks=coastTicks=0;direction=opening?FWD:REV;
  operationStart=motionStart=millis();mode=opening?OPENING:CLOSING;
  Serial.println(opening?"\nMOUTH OPENING":"\nMOUTH CLOSING");
  Serial.print("Start: ");Serial.print(pos);
  Serial.print(" | cutoff: ");Serial.println(opening?openCutoff():closeCutoff());
  motorOn(direction);return true;
}
void startTalking(){
  if(mode!=IDLE||talking){Serial.println("Busy.");return;}
  if(!synced){Serial.println("Use H first.");return;}
  if(abs(distance(pos,CLOSED))>6){Serial.println("Move near 140 first.");return;}
  talking=true;talkCycles=0;talkStart=millis();nextTalk=NEXT_OPEN;
  Serial.println("\n=== ADAPTIVE TALKING TEST: 20 CYCLES ===");
  Serial.print("Initial cutoffs OPEN/CLOSE: ");Serial.print(openCutoff());Serial.print(" / ");Serial.println(closeCutoff());
}
void finishMouth(bool opening){
  const uint32_t now=millis();
  const int goal=opening?OPEN:CLOSED;
  const int error=distance(pos,goal); // target - actual; positive means too short in forward direction
  const int tolerance=opening?OPEN_TOL:CLOSE_TOL;
  const int oldCutoff=opening?openCutoff():closeCutoff();
  Serial.println(opening?"--- OPEN RESULT ---":"--- CLOSE RESULT ---");
  Serial.print("Cutoff: ");Serial.print(cutoffPos);
  Serial.print(" | powered ticks: ");Serial.print(poweredTicks);
  Serial.print(" | coast ticks: ");Serial.println(coastTicks);
  Serial.print("Final: ");Serial.print(pos);
  Serial.print(" | target: ");Serial.print(goal);
  Serial.print(" | signed error: ");Serial.println(error);
  Serial.print("Powered ms: ");Serial.print(motionCutoffAt-motionStart);
  Serial.print(" | last coast tick ms: ");Serial.print(coastTicks?lastCoastAt-motionCutoffAt:0);
  Serial.print(" | quiet ms: ");Serial.print(now-lastTickAt);
  Serial.print(" | total ms: ");Serial.println(now-motionStart);
  mode=IDLE;direction=STOP;settling=NONE;
  if(!talking)return;
  if(coastTicks<MIN_COAST_TICKS){abortMotion("too few coast ticks");return;}
  if(abs(error)>tolerance){abortMotion("mouth outside allowed range");return;}
  // Conservative integral correction: one cutoff tick per completed motion,
  // with a deadband of +/-2 ticks. Independent corrections for each direction.
  // For both directions: positive error means stop too short in the forward
  // coordinate, so move the cutoff forward by one tick; negative means backward.
  int change=error>2?1:(error< -2?-1:0);
  if(opening)openAdjust=constrain(openAdjust+change,-MAX_ADJUST,MAX_ADJUST);
  else closeAdjust=constrain(closeAdjust+change,-MAX_ADJUST,MAX_ADJUST);
  Serial.print("Adaptive cutoff: ");Serial.print(oldCutoff);
  Serial.print(" -> ");Serial.println(opening?openCutoff():closeCutoff());
  if(opening)nextTalk=NEXT_CLOSE;
  else{
    ++talkCycles;Serial.print("Completed cycle ");Serial.print(talkCycles);Serial.print("/");Serial.println(TALK_CYCLES);
    if(talkCycles>=TALK_CYCLES){talking=false;nextTalk=NO_NEXT;Serial.print("TALK COMPLETE, ms: ");Serial.println(millis()-talkStart);}
    else nextTalk=NEXT_OPEN;
  }
}
void processTalk(){
  if(!talking||mode!=IDLE||nextTalk==NO_NEXT)return;
  Next n=nextTalk;nextTalk=NO_NEXT;
  if(!startMouth(n==NEXT_OPEN))abortMotion("could not start next mouth movement");
}
void status(){
  Serial.print("Synced: ");Serial.print(synced?"YES":"NO");
  Serial.print(" | position: ");Serial.print(pos);
  Serial.print(" | total ticks: ");Serial.print(totalTicks);
  Serial.print(" | cutoffs OPEN/CLOSE: ");Serial.print(openCutoff());Serial.print("/");Serial.println(closeCutoff());
}
void handle(char *s){
  if((s[0]=='X'||s[0]=='x')&&s[1]=='\0'){abortMotion("manual stop");return;}
  if((s[0]=='S'||s[0]=='s')&&s[1]=='\0'){status();return;}
  if((s[0]=='H'||s[0]=='h')&&s[1]=='\0'){startHome();return;}
  if((s[0]=='T'||s[0]=='t')&&s[1]=='\0'){startTalking();return;}
  if((s[0]=='O'||s[0]=='o')&&s[1]=='\0'){
    if(!startMouth(true))Serial.println("Cannot open: busy, not homed, or wrong starting position.");return;
  }
  if((s[0]=='C'||s[0]=='c')&&s[1]=='\0'){
    if(!startMouth(false))Serial.println("Cannot close: busy, not homed, or wrong starting position.");return;
  }
  if(isdigit((unsigned char)s[0])){
    char* end;long v=strtol(s,&end,10);
    if(*end=='\0'&&v>=0&&v<REV_TICKS){startMove((int)v);return;}
  }
  Serial.println("Unknown command.");
}
void readCommands(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\r'||c=='\n'){
      if(commandLength){command[commandLength]='\0';handle(command);commandLength=0;}
    }else if(commandLength<sizeof(command)-1)command[commandLength++]=c;
  }
}
void setup(){
  Serial.begin(115200);
  digitalWrite(MOTOR_FWD_PIN,LOW);digitalWrite(MOTOR_REV_PIN,LOW);
  pinMode(MOTOR_FWD_PIN,OUTPUT);pinMode(MOTOR_REV_PIN,OUTPUT);
  digitalWrite(POWER_DOWN_PIN,HIGH);pinMode(POWER_DOWN_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);pinMode(CAM_HOME_PIN,INPUT_PULLUP);pinMode(GEAR_ROTATION_PIN,INPUT_PULLUP);
  motorOff();lastGear=digitalRead(GEAR_ROTATION_PIN);lastHome=digitalRead(CAM_HOME_PIN);homeArmed=(lastHome==HIGH);
  Serial.println("FURBY ADAPTIVE MOUTH CONTROLLER - TIMING FIX");
  Serial.println("H=home, 140=position, T=20 cycles, O=open, C=close, S=status, X=stop");
}
void loop(){
  encoder();
  homeSensor();
  readCommands();

  // Read time at each check. Never reuse a timestamp captured before
  // beginSettle(), which records a NEWER motorOffAt timestamp.
  if(mode==HOMING && (uint32_t)(millis()-operationStart)>=HOME_TIMEOUT_MS){
    abortMotion("HOME timeout");
    return;
  }

  if(mode==PULSING && (uint32_t)(millis()-pulseStart)>=pulseLength){
    beginSettle(MOVE);
    return;  // First settling check occurs in a future loop iteration.
  }

  if(mode==OPENING){
    if(pos>=openCutoff()){
      cutoffPos=pos;
      motionCutoffAt=millis();
      beginSettle(OPEN_DONE);
      return;
    }
    if((uint32_t)(millis()-operationStart)>=MOTION_TIMEOUT_MS){
      abortMotion("opening timeout");
      return;
    }
  }

  if(mode==CLOSING){
    if(pos<=closeCutoff()){
      cutoffPos=pos;
      motionCutoffAt=millis();
      beginSettle(CLOSE_DONE);
      return;
    }
    if((uint32_t)(millis()-operationStart)>=MOTION_TIMEOUT_MS){
      abortMotion("closing timeout");
      return;
    }
  }

  if(mode==SETTLING){
    // Fresh timestamp, taken AFTER the motor-off timestamp exists.
    const uint32_t settleNow=millis();
    const uint32_t elapsed=(uint32_t)(settleNow-motorOffAt);
    const uint32_t quietFor=(uint32_t)(settleNow-lastTickAt);
    const bool fast=talking && (settling==OPEN_DONE || settling==CLOSE_DONE);
    const uint32_t quietRequired=fast?FAST_QUIET_MS:NORMAL_QUIET_MS;
    const bool quiet=quietFor>=quietRequired;
    const bool minElapsed=!fast || elapsed>=MIN_FAST_COAST_MS;

    if(elapsed>=MAX_SETTLE_MS){
      abortMotion("settling timeout");
      return;
    }

    if(quiet && minElapsed){
      const Settle completed=settling;
      settling=NONE;
      if(completed==HOME){
        mode=IDLE;
        direction=STOP;
        Serial.print("HOME COMPLETE. Position: ");
        Serial.println(pos);
      }else if(completed==MOVE){
        mode=MOVING;
      }else if(completed==OPEN_DONE){
        finishMouth(true);
      }else if(completed==CLOSE_DONE){
        finishMouth(false);
      }
      return; // Do not start the next motion in this iteration.
    }
  }

  if(mode==MOVING)continueMove();
  processTalk();
}
