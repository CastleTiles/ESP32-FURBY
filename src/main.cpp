
#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>

// ============================================================
// FURBY TALKING CONTROLLER - RELIABILITY TEST
// ESP32 + Original Furby Motor / Optical Encoder
// ============================================================

// PIN ASSIGNMENTS
#define BUTTON_PIN         14
#define MOTOR_FWD_PIN      16
#define MOTOR_REV_PIN      17
#define POWER_DOWN_PIN     32
#define CAM_HOME_PIN       13
#define GEAR_ROTATION_PIN  33

// CAM POSITIONS
const int TICKS_PER_REV = 208;
const int MOUTH_CLOSED = 140;
const int MOUTH_OPEN = 175;

// CALIBRATED MOTOR CUTOFFS
const int OPEN_CUTOFF = 154;
const int CLOSE_CUTOFF = 160;

// POSITIONING
const unsigned long LARGE_PULSE_MS = 50;
const unsigned long SMALL_PULSE_MS = 20;
const int POSITION_TOLERANCE = 3;
const int MAX_MOVE_PULSES = 80;

// SETTLING
const unsigned long NORMAL_SETTLE_MS = 500;
const unsigned long FAST_SETTLE_MS = 100;
const unsigned long MIN_FAST_COAST_MS = 300;
const unsigned long MAX_SETTLE_MS = 2000;

// TIMEOUTS
const unsigned long HOME_TIMEOUT_MS = 5000;
const unsigned long MOVE_TIMEOUT_MS = 10000;
const unsigned long MOTION_TIMEOUT_MS = 1500;

// TALKING
const int TALK_CYCLES = 4;

// POSITION VALIDATION
const int OPEN_TOLERANCE = 8;
const int CLOSE_TOLERANCE = 6;

// Minimum observed coast ticks during automatic talking.
// Our previous successful runs typically produced 19-24.
const unsigned long MIN_TALK_COAST_TICKS = 8;

// ============================================================
// STATES
// ============================================================

enum Direction {
    DIR_STOP = 0,
    DIR_FORWARD = 1,
    DIR_REVERSE = -1
};

enum Mode {
    MODE_IDLE,
    MODE_HOMING,
    MODE_MOVING,
    MODE_PULSING,
    MODE_OPENING,
    MODE_CLOSING,
    MODE_SETTLING
};

enum SettleReason {
    SETTLE_NONE,
    SETTLE_HOME,
    SETTLE_MOVE,
    SETTLE_OPEN,
    SETTLE_CLOSE
};

enum TalkPhase {
    TALK_NONE,
    TALK_START_OPEN,
    TALK_START_CLOSE
};

Mode mode = MODE_IDLE;
SettleReason settleReason = SETTLE_NONE;
Direction activeDirection = DIR_STOP;
TalkPhase pendingTalkPhase = TALK_NONE;

// POSITION
int position = 0;
int targetPosition = 0;
int cutoffPosition = 0;
bool synchronized = false;

// ENCODER
bool lastGearState = HIGH;
unsigned long totalTicks = 0;
unsigned long pulseTicks = 0;
unsigned long coastTicks = 0;

// HOME
bool lastHomeState = HIGH;
bool homeArmed = false;

// TIMING
unsigned long operationStart = 0;
unsigned long pulseStart = 0;
unsigned long motorOffTime = 0;
unsigned long lastTickTime = 0;
unsigned long lastCoastTickTime = 0;
unsigned long motionStart = 0;
unsigned long motionCutoffTime = 0;
unsigned long currentPulseDuration = SMALL_PULSE_MS;

int pulseCount = 0;

// TALKING
bool talking = false;
int talkCycle = 0;
unsigned long talkStart = 0;

// SERIAL
char inputBuffer[24];
uint8_t inputLength = 0;

// ============================================================
// POSITION MATH
// ============================================================

int normalizePosition(int p) {
    p %= TICKS_PER_REV;

    if (p < 0) {
        p += TICKS_PER_REV;
    }

    return p;
}

int signedDistance(int from, int to) {
    int delta = normalizePosition(to - from);

    if (delta > TICKS_PER_REV / 2) {
        delta -= TICKS_PER_REV;
    }

    return delta;
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorOff() {
    digitalWrite(MOTOR_FWD_PIN, LOW);
    digitalWrite(MOTOR_REV_PIN, LOW);
}

void motorOn(Direction dir) {
    motorOff();

    if (dir == DIR_FORWARD) {
        digitalWrite(MOTOR_FWD_PIN, HIGH);
    }
    else if (dir == DIR_REVERSE) {
        digitalWrite(MOTOR_REV_PIN, HIGH);
    }
}

// ============================================================
// STOP / ABORT
// ============================================================

void stopEverything() {
    motorOff();

    mode = MODE_IDLE;
    settleReason = SETTLE_NONE;
    activeDirection = DIR_STOP;

    talking = false;
    pendingTalkPhase = TALK_NONE;

    synchronized = false;

    Serial.println();
    Serial.println("STOPPED - POSITION UNCERTAIN");
    Serial.println("Use H to re-home.");
}

void abortTalking(const char *reason) {
    motorOff();

    Serial.println();
    Serial.println("!!! TALKING ABORTED !!!");
    Serial.println(reason);

    Serial.print("Estimated position: ");
    Serial.println(position);

    talking = false;
    pendingTalkPhase = TALK_NONE;

    mode = MODE_IDLE;
    settleReason = SETTLE_NONE;
    activeDirection = DIR_STOP;

    synchronized = false;

    Serial.println("Re-home with H before continuing.");
}

// ============================================================
// ENCODER
// ============================================================

void updateEncoder() {
    bool current = digitalRead(GEAR_ROTATION_PIN);

    if (lastGearState == HIGH && current == LOW) {
        totalTicks++;

        if (mode == MODE_PULSING ||
            mode == MODE_OPENING ||
            mode == MODE_CLOSING) {
            pulseTicks++;
        }
        else if (mode == MODE_SETTLING) {
            coastTicks++;
            lastCoastTickTime = millis();
        }

        // Single-channel encoder:
        // Direction is inferred from commanded motor direction.
        if (synchronized && activeDirection != DIR_STOP) {
            position = normalizePosition(
                position + (int)activeDirection
            );
        }

        lastTickTime = millis();
    }

    lastGearState = current;
}

// ============================================================
// HOME SENSOR
// ============================================================

void updateHome() {
    bool current = digitalRead(CAM_HOME_PIN);

    if (current == HIGH) {
        homeArmed = true;
    }

    if (lastHomeState == HIGH &&
        current == LOW &&
        homeArmed) {

        homeArmed = false;

        if (mode == MODE_HOMING) {
            position = 0;
            synchronized = true;

            Serial.println("HOME detected.");

            motorOff();

            motorOffTime = millis();
            lastTickTime = motorOffTime;
            lastCoastTickTime = motorOffTime;

            activeDirection = DIR_FORWARD;

            settleReason = SETTLE_HOME;
            mode = MODE_SETTLING;
        }
    }

    lastHomeState = current;
}

// ============================================================
// HOMING
// ============================================================

void startHoming() {
    if (mode != MODE_IDLE) {
        Serial.println("Busy.");
        return;
    }

    synchronized = false;
    talking = false;
    pendingTalkPhase = TALK_NONE;

    pulseTicks = 0;
    coastTicks = 0;

    activeDirection = DIR_FORWARD;
    operationStart = millis();

    settleReason = SETTLE_NONE;
    mode = MODE_HOMING;

    Serial.println();
    Serial.println("Searching for HOME...");

    motorOn(DIR_FORWARD);
}

// ============================================================
// POSITIONING
// ============================================================

void startPulse(Direction dir, unsigned long duration) {
    activeDirection = dir;

    pulseTicks = 0;
    coastTicks = 0;

    currentPulseDuration = duration;
    pulseStart = millis();
    lastTickTime = pulseStart;

    mode = MODE_PULSING;
    motorOn(dir);

    Serial.print("Pulse ");
    Serial.print(dir == DIR_FORWARD ? "FWD " : "REV ");
    Serial.print(duration);
    Serial.println(" ms");
}

void startMove(int target) {
    if (mode != MODE_IDLE) {
        Serial.println("Busy.");
        return;
    }

    if (!synchronized) {
        Serial.println("Use H first.");
        return;
    }

    talking = false;
    pendingTalkPhase = TALK_NONE;

    targetPosition = normalizePosition(target);
    operationStart = millis();
    pulseCount = 0;

    mode = MODE_MOVING;

    Serial.print("Moving to ");
    Serial.println(targetPosition);
}

void continueMove() {
    if (mode != MODE_MOVING) {
        return;
    }

    int delta = signedDistance(position, targetPosition);
    int distance = abs(delta);

    if (distance <= POSITION_TOLERANCE) {
        Serial.print("TARGET REACHED: ");
        Serial.println(position);

        mode = MODE_IDLE;
        activeDirection = DIR_STOP;
        return;
    }

    if (pulseCount >= MAX_MOVE_PULSES ||
        millis() - operationStart >= MOVE_TIMEOUT_MS) {
        Serial.println("MOVE TIMEOUT");
        stopEverything();
        return;
    }

    Direction dir =
        delta > 0 ? DIR_FORWARD : DIR_REVERSE;

    unsigned long duration =
        distance > 20 ? LARGE_PULSE_MS : SMALL_PULSE_MS;

    pulseCount++;
    startPulse(dir, duration);
}

// ============================================================
// CONTINUOUS MOUTH MOTIONS
// ============================================================

bool startOpening() {
    if (mode != MODE_IDLE || !synchronized) {
        return false;
    }

    if (abs(signedDistance(position, MOUTH_CLOSED)) > 5) {
        Serial.println("Move near 140 first.");
        return false;
    }

    pulseTicks = 0;
    coastTicks = 0;

    activeDirection = DIR_FORWARD;
    operationStart = millis();
    motionStart = operationStart;

    settleReason = SETTLE_NONE;
    mode = MODE_OPENING;

    Serial.println();
    Serial.println("MOUTH OPENING");

    Serial.print("Start: ");
    Serial.println(position);

    motorOn(DIR_FORWARD);
    return true;
}

bool startClosing() {
    if (mode != MODE_IDLE || !synchronized) {
        return false;
    }

    if (abs(signedDistance(position, MOUTH_OPEN)) > 8) {
        Serial.println("Move near 175 first.");
        return false;
    }

    pulseTicks = 0;
    coastTicks = 0;

    activeDirection = DIR_REVERSE;
    operationStart = millis();
    motionStart = operationStart;

    settleReason = SETTLE_NONE;
    mode = MODE_CLOSING;

    Serial.println();
    Serial.println("MOUTH CLOSING");

    Serial.print("Start: ");
    Serial.println(position);

    motorOn(DIR_REVERSE);
    return true;
}

// ============================================================
// TALKING
// ============================================================

void startTalking() {
    if (mode != MODE_IDLE) {
        Serial.println("Busy.");
        return;
    }

    if (!synchronized) {
        Serial.println("Use H first.");
        return;
    }

    if (abs(signedDistance(position, MOUTH_CLOSED)) > 5) {
        Serial.println("Move near 140 first.");
        return;
    }

    talking = true;
    talkCycle = 0;
    talkStart = millis();

    pendingTalkPhase = TALK_START_OPEN;

    Serial.println();
    Serial.println("============================");
    Serial.println("FURBY RELIABLE TALKING TEST");
    Serial.println("============================");

    Serial.println("Cycles: 4");
}

// ============================================================
// MOTION RESULT / VALIDATION
// ============================================================

void finishMotion(bool opening) {
    unsigned long now = millis();

    unsigned long poweredTime =
        motionCutoffTime - motionStart;

    unsigned long coastDuration =
        coastTicks > 0
            ? lastCoastTickTime - motionCutoffTime
            : 0;

    unsigned long waitAfterLastTick =
        now - lastCoastTickTime;

    int target = opening ? MOUTH_OPEN : MOUTH_CLOSED;
    int tolerance = opening ? OPEN_TOLERANCE : CLOSE_TOLERANCE;

    int error = signedDistance(position, target);

    Serial.println();
    Serial.println(
        opening ? "--- OPEN RESULT ---" : "--- CLOSE RESULT ---"
    );

    Serial.print("Cutoff position: ");
    Serial.println(cutoffPosition);

    Serial.print("Powered ticks: ");
    Serial.println(pulseTicks);

    Serial.print("Coast ticks: ");
    Serial.println(coastTicks);

    Serial.print("Final position: ");
    Serial.println(position);

    Serial.print("Position error: ");
    Serial.println(error);

    Serial.print("Motor powered: ");
    Serial.print(poweredTime);
    Serial.println(" ms");

    Serial.print("Last coast tick after cutoff: ");
    Serial.print(coastDuration);
    Serial.println(" ms");

    Serial.print("Wait after last tick: ");
    Serial.print(waitAfterLastTick);
    Serial.println(" ms");

    Serial.print("Total movement: ");
    Serial.print(now - motionStart);
    Serial.println(" ms");

    Serial.println("--------------------");

    // Finish the current motion completely.
    mode = MODE_IDLE;
    activeDirection = DIR_STOP;
    settleReason = SETTLE_NONE;

    if (!talking) {
        return;
    }

    // Validate the completed motion.
    if (coastTicks < MIN_TALK_COAST_TICKS) {
        abortTalking("Insufficient coast ticks.");
        return;
    }

    if (abs(error) > tolerance) {
        abortTalking("Mouth stopped outside target tolerance.");
        return;
    }

    // Queue the next motion.
    // Do NOT start it inside this function.
    if (opening) {
        pendingTalkPhase = TALK_START_CLOSE;
    }
    else {
        talkCycle++;

        Serial.print("Completed cycle ");
        Serial.print(talkCycle);
        Serial.print("/");
        Serial.println(TALK_CYCLES);

        if (talkCycle >= TALK_CYCLES) {
            talking = false;
            pendingTalkPhase = TALK_NONE;

            Serial.println();
            Serial.println("TALKING COMPLETE");

            Serial.print("Total animation time: ");
            Serial.print(millis() - talkStart);
            Serial.println(" ms");
        }
        else {
            pendingTalkPhase = TALK_START_OPEN;
        }
    }
}

// ============================================================
// PROCESS QUEUED TALK MOTION
// ============================================================

void processTalking() {
    if (!talking ||
        mode != MODE_IDLE ||
        pendingTalkPhase == TALK_NONE) {
        return;
    }

    TalkPhase next = pendingTalkPhase;
    pendingTalkPhase = TALK_NONE;

    bool started = false;

    if (next == TALK_START_OPEN) {
        started = startOpening();
    }
    else if (next == TALK_START_CLOSE) {
        started = startClosing();
    }

    if (!started) {
        abortTalking("Unable to start next mouth movement.");
    }
}

// ============================================================
// STATUS
// ============================================================

void printStatus() {
    Serial.println();
    Serial.println("--------- STATUS ---------");

    Serial.print("Synchronized: ");
    Serial.println(synchronized ? "YES" : "NO");

    Serial.print("Estimated position: ");
    Serial.println(position);

    Serial.print("Encoder ticks: ");
    Serial.println(totalTicks);

    Serial.print("Talking: ");
    Serial.println(talking ? "YES" : "NO");

    Serial.print("Mode: ");
    Serial.println((int)mode);

    Serial.println("--------------------------");
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void processCommand(char *command) {
    if (command[0] == 'X' || command[0] == 'x') {
        stopEverything();
        return;
    }

    if (command[0] == 'S' || command[0] == 's') {
        printStatus();
        return;
    }

    if (command[0] == 'H' || command[0] == 'h') {
        startHoming();
        return;
    }

    if (command[0] == 'O' || command[0] == 'o') {
        if (mode == MODE_IDLE && !talking) {
            startOpening();
        } else {
            Serial.println("Busy.");
        }
        return;
    }

    if (command[0] == 'C' || command[0] == 'c') {
        if (mode == MODE_IDLE && !talking) {
            startClosing();
        } else {
            Serial.println("Busy.");
        }
        return;
    }

    if (command[0] == 'T' || command[0] == 't') {
        startTalking();
        return;
    }

    if (isdigit((unsigned char)command[0])) {
        char *end;
        long value = strtol(command, &end, 10);

        if (*end == '\0' &&
            value >= 0 &&
            value < TICKS_PER_REV) {
            startMove((int)value);
            return;
        }
    }

    Serial.println("Unknown command.");
}

void readSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (inputLength > 0) {
                inputBuffer[inputLength] = '\0';
                processCommand(inputBuffer);
                inputLength = 0;
            }
        }
        else if (inputLength < sizeof(inputBuffer) - 1) {
            inputBuffer[inputLength++] = c;
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(CAM_HOME_PIN, INPUT_PULLUP);
    pinMode(GEAR_ROTATION_PIN, INPUT_PULLUP);

    digitalWrite(MOTOR_FWD_PIN, LOW);
    digitalWrite(MOTOR_REV_PIN, LOW);
    digitalWrite(POWER_DOWN_PIN, HIGH);

    pinMode(MOTOR_FWD_PIN, OUTPUT);
    pinMode(MOTOR_REV_PIN, OUTPUT);
    pinMode(POWER_DOWN_PIN, OUTPUT);

    motorOff();

    lastGearState = digitalRead(GEAR_ROTATION_PIN);
    lastHomeState = digitalRead(CAM_HOME_PIN);
    homeArmed = lastHomeState == HIGH;

    Serial.println();
    Serial.println("================================");
    Serial.println("FURBY RELIABLE TALKING CONTROLLER");
    Serial.println("================================");

    Serial.println("H = Home");
    Serial.println("140 = Closed-mouth positioning");
    Serial.println("O = Smooth opening");
    Serial.println("C = Smooth closing");
    Serial.println("T = Reliable talking animation");
    Serial.println("S = Status");
    Serial.println("X = Stop");

    Serial.println();
    Serial.println("Ready.");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    updateEncoder();
    updateHome();
    readSerial();

    unsigned long now = millis();

    // HOME TIMEOUT
    if (mode == MODE_HOMING &&
        now - operationStart >= HOME_TIMEOUT_MS) {
        Serial.println("HOME TIMEOUT");
        stopEverything();
    }

    // POSITIONING PULSE
    if (mode == MODE_PULSING &&
        now - pulseStart >= currentPulseDuration) {

        motorOff();

        motorOffTime = millis();
        lastTickTime = motorOffTime;
        lastCoastTickTime = motorOffTime;

        settleReason = SETTLE_MOVE;
        mode = MODE_SETTLING;
    }

    // CONTINUOUS OPENING
    if (mode == MODE_OPENING) {
        if (position >= OPEN_CUTOFF) {

            motorOff();

            cutoffPosition = position;
            motionCutoffTime = millis();
            motorOffTime = motionCutoffTime;
            lastTickTime = motorOffTime;
            lastCoastTickTime = motorOffTime;

            settleReason = SETTLE_OPEN;
            mode = MODE_SETTLING;
        }
        else if (now - operationStart >= MOTION_TIMEOUT_MS) {
            Serial.println("OPEN TIMEOUT");
            stopEverything();
        }
    }

    // CONTINUOUS CLOSING
    if (mode == MODE_CLOSING) {
        if (position <= CLOSE_CUTOFF) {

            motorOff();

            cutoffPosition = position;
            motionCutoffTime = millis();
            motorOffTime = motionCutoffTime;
            lastTickTime = motorOffTime;
            lastCoastTickTime = motorOffTime;

            settleReason = SETTLE_CLOSE;
            mode = MODE_SETTLING;
        }
        else if (now - operationStart >= MOTION_TIMEOUT_MS) {
            Serial.println("CLOSE TIMEOUT");
            stopEverything();
        }
    }

    // SETTLING
    if (mode == MODE_SETTLING) {

        unsigned long elapsed = now - motorOffTime;

        bool fastTalkSettle =
            talking &&
            (settleReason == SETTLE_OPEN ||
             settleReason == SETTLE_CLOSE);

        unsigned long requiredQuiet =
            fastTalkSettle
                ? FAST_SETTLE_MS
                : NORMAL_SETTLE_MS;

        bool quietEnough =
            now - lastTickTime >= requiredQuiet;

        bool minimumCoastElapsed =
            !fastTalkSettle ||
            elapsed >= MIN_FAST_COAST_MS;

        bool maximumElapsed =
            elapsed >= MAX_SETTLE_MS;

        if ((quietEnough && minimumCoastElapsed) ||
            maximumElapsed) {

            SettleReason completedReason = settleReason;

            // Clear the settling state BEFORE processing
            // the completed movement.
            settleReason = SETTLE_NONE;

            if (completedReason == SETTLE_HOME) {

                Serial.print("HOME COMPLETE. Position: ");
                Serial.println(position);

                mode = MODE_IDLE;
                activeDirection = DIR_STOP;
            }
            else if (completedReason == SETTLE_MOVE) {
                mode = MODE_MOVING;
            }
            else if (completedReason == SETTLE_OPEN) {
                finishMotion(true);
            }
            else if (completedReason == SETTLE_CLOSE) {
                finishMotion(false);
            }
        }
    }

    // POSITIONING
    if (mode == MODE_MOVING) {
        continueMove();
    }

    // TALKING - starts only after prior movement is finished.
    processTalking();
}
