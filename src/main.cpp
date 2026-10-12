
#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

// ============================================================
// FURBY SERIAL POSITIONING TOOL - DIAGNOSTIC VERSION
// ============================================================

// PIN ASSIGNMENTS
#define BUTTON_PIN         14
#define MOTOR_FWD_PIN      16
#define MOTOR_REV_PIN      17
#define POWER_DOWN_PIN     32
#define CAM_HOME_PIN       13
#define GEAR_ROTATION_PIN  33

// SETTINGS
const long TICKS_PER_REV = 208;
const long COAST_COMPENSATION = 29;

const unsigned long HOME_DEBOUNCE_MS = 10;
const unsigned long SETTLE_MS = 500;
const unsigned long MAX_SETTLE_MS = 3000;
const unsigned long MOTOR_TIMEOUT_MS = 5000;
const unsigned long REPORT_INTERVAL_MS = 500;

// STATES
enum State {
    UNHOMED,
    HOMING,
    HOMING_SETTLE,
    IDLE,
    MOVING,
    MOVE_SETTLE,
    ERROR_STATE
};

State state = UNHOMED;

// POSITION
long position = 0;
long targetPosition = 0;
long cutoffPosition = 0;
long positionAtMotorOff = 0;

bool positionValid = false;
bool pendingMove = false;

// TIMING
unsigned long motorStartTime = 0;
unsigned long motorOffTime = 0;
unsigned long lastGearTickTime = 0;
unsigned long lastReportTime = 0;

// ENCODER
bool lastGearState = HIGH;
unsigned long encoderTicksThisRun = 0;

// CAM HOME
bool lastRawHome = HIGH;
bool stableHome = HIGH;
bool homeArmed = false;
bool homeEvent = false;

unsigned long homeChangeTime = 0;
unsigned long rawHomeTransitions = 0;
unsigned long validHomeTransitions = 0;

// SERIAL INPUT
char inputBuffer[24];
uint8_t inputLength = 0;

// ------------------------------------------------------------
// MOTOR
// ------------------------------------------------------------

void motorOff() {
    digitalWrite(MOTOR_FWD_PIN, LOW);
    digitalWrite(MOTOR_REV_PIN, LOW);
}

void startMotor() {
    motorOff();

    encoderTicksThisRun = 0;
    rawHomeTransitions = 0;
    validHomeTransitions = 0;

    motorStartTime = millis();
    lastReportTime = motorStartTime;

    digitalWrite(MOTOR_FWD_PIN, HIGH);
}

void stopMotor() {
    motorOff();
    motorOffTime = millis();
    lastGearTickTime = motorOffTime;
    positionAtMotorOff = position;
}

// ------------------------------------------------------------
// POSITION HELPERS
// ------------------------------------------------------------

long wrappedPosition(long p) {
    p %= TICKS_PER_REV;
    if (p < 0) p += TICKS_PER_REV;
    return p;
}

bool motorRunning() {
    return state == HOMING || state == MOVING;
}

// ------------------------------------------------------------
// DIAGNOSTIC REPORT
// ------------------------------------------------------------

void reportDiagnostics() {
    Serial.print("[DIAG] Time: ");
    Serial.print(millis() - motorStartTime);

    Serial.print(" ms | Encoder ticks: ");
    Serial.print(encoderTicksThisRun);

    Serial.print(" | Position: ");
    Serial.print(position);

    Serial.print(" | HOME raw: ");
    Serial.print(digitalRead(CAM_HOME_PIN));

    Serial.print(" | HOME stable: ");
    Serial.print(stableHome);

    Serial.print(" | HOME armed: ");
    Serial.print(homeArmed ? "YES" : "NO");

    Serial.print(" | HOME raw changes: ");
    Serial.print(rawHomeTransitions);

    Serial.print(" | HOME valid changes: ");
    Serial.println(validHomeTransitions);
}

// ------------------------------------------------------------
// STATUS
// ------------------------------------------------------------

void printStatus() {
    Serial.println();
    Serial.println("------ STATUS ------");

    Serial.print("State: ");

    switch (state) {
        case UNHOMED: Serial.println("UNHOMED"); break;
        case HOMING: Serial.println("HOMING"); break;
        case HOMING_SETTLE: Serial.println("HOMING_SETTLE"); break;
        case IDLE: Serial.println("IDLE"); break;
        case MOVING: Serial.println("MOVING"); break;
        case MOVE_SETTLE: Serial.println("MOVE_SETTLE"); break;
        case ERROR_STATE: Serial.println("ERROR"); break;
    }

    Serial.print("Position valid: ");
    Serial.println(positionValid ? "YES" : "NO");

    Serial.print("Raw position: ");
    Serial.println(position);

    Serial.print("Wrapped position: ");
    Serial.println(wrappedPosition(position));

    Serial.print("HOME stable: ");
    Serial.println(stableHome);

    Serial.print("Encoder ticks last run: ");
    Serial.println(encoderTicksThisRun);

    Serial.println("--------------------");
}

// ------------------------------------------------------------
// STOP / ERROR
// ------------------------------------------------------------

void emergencyStop() {
    motorOff();
    pendingMove = false;
    homeEvent = false;
    positionValid = false;
    state = UNHOMED;

    Serial.println();
    Serial.println("MOTOR STOPPED");
    Serial.println("Type H to re-home.");
}

void motorError(const char *message) {
    motorOff();

    pendingMove = false;
    positionValid = false;
    state = ERROR_STATE;

    Serial.println();
    Serial.print("ERROR: ");
    Serial.println(message);

    reportDiagnostics();

    Serial.println("Type H to re-home.");
}

// ------------------------------------------------------------
// SETTLING
// ------------------------------------------------------------

bool settled(unsigned long now) {
    if (now - lastGearTickTime >= SETTLE_MS) {
        return true;
    }

    if (now - motorOffTime >= MAX_SETTLE_MS) {
        Serial.println("WARNING: Maximum settle time reached.");
        return true;
    }

    return false;
}

// ------------------------------------------------------------
// HOME
// ------------------------------------------------------------

void beginHome() {
    if (motorRunning() ||
        state == HOMING_SETTLE ||
        state == MOVE_SETTLE) {

        Serial.println("Busy. Type X to stop.");
        return;
    }

    positionValid = false;
    pendingMove = false;
    homeEvent = false;

    homeArmed = (stableHome == HIGH);

    Serial.println();
    Serial.println("SEARCHING FOR HOME...");

    startMotor();
    state = HOMING;
}

// ------------------------------------------------------------
// MOVE
// ------------------------------------------------------------

void beginMove(long requested) {
    if (state != IDLE || !positionValid) {
        Serial.println("Not ready. Type H to home.");
        return;
    }

    targetPosition = requested;
    cutoffPosition = targetPosition - COAST_COMPENSATION;

    if (position >= cutoffPosition) {

        Serial.println();
        Serial.println("Cutoff is behind current position.");
        Serial.println("Returning through HOME...");

        pendingMove = true;
        positionValid = false;
        homeEvent = false;
        homeArmed = (stableHome == HIGH);

        startMotor();
        state = HOMING;
        return;
    }

    Serial.println();
    Serial.println("MOVING TO POSITION");

    Serial.print("Current: ");
    Serial.println(position);

    Serial.print("Target: ");
    Serial.println(targetPosition);

    Serial.print("Cutoff: ");
    Serial.println(cutoffPosition);

    startMotor();
    state = MOVING;
}

// ------------------------------------------------------------
// SERIAL COMMANDS
// ------------------------------------------------------------

void processCommand(char *command) {

    while (*command &&
           isspace((unsigned char)*command)) {
        command++;
    }

    size_t len = strlen(command);

    while (len > 0 &&
           isspace((unsigned char)command[len - 1])) {
        command[--len] = '\0';
    }

    if (len == 0) return;

    if ((command[0] == 'H' || command[0] == 'h') &&
        command[1] == '\0') {
        beginHome();
        return;
    }

    if ((command[0] == 'S' || command[0] == 's') &&
        command[1] == '\0') {
        printStatus();
        return;
    }

    if ((command[0] == 'X' || command[0] == 'x') &&
        command[1] == '\0') {
        emergencyStop();
        return;
    }

    char *endPtr;
    long requested = strtol(command, &endPtr, 10);

    if (endPtr == command || *endPtr != '\0') {
        Serial.println("Unknown command.");
        return;
    }

    if (requested < 0 || requested >= TICKS_PER_REV) {
        Serial.println("Enter position 0-207.");
        return;
    }

    beginMove(requested);
}

void readSerial() {
    while (Serial.available() > 0) {

        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (inputLength > 0) {
                inputBuffer[inputLength] = '\0';
                processCommand(inputBuffer);
                inputLength = 0;
            }
        } else if (inputLength < sizeof(inputBuffer) - 1) {
            inputBuffer[inputLength++] = c;
        } else {
            inputLength = 0;
            Serial.println("Input too long.");
        }
    }
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

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
    lastGearTickTime = millis();

    lastRawHome = digitalRead(CAM_HOME_PIN);
    stableHome = lastRawHome;
    homeChangeTime = millis();
    homeArmed = (stableHome == HIGH);

    Serial.println();
    Serial.println("================================");
    Serial.println("FURBY POSITION TOOL - DIAGNOSTICS");
    Serial.println("================================");

    Serial.println("Commands:");
    Serial.println(" H      Home");
    Serial.println(" 0-207  Move to position");
    Serial.println(" S      Status");
    Serial.println(" X      Stop");

    Serial.println();
    Serial.println("Type H to begin.");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop() {
    unsigned long now = millis();

    // ENCODER
    bool gearState = digitalRead(GEAR_ROTATION_PIN);

    if (lastGearState == HIGH && gearState == LOW) {
        position++;
        encoderTicksThisRun++;
        lastGearTickTime = now;
    }

    lastGearState = gearState;

    // CAM HOME
    bool rawHome = digitalRead(CAM_HOME_PIN);

    if (rawHome != lastRawHome) {
        homeChangeTime = now;
        rawHomeTransitions++;
    }

    lastRawHome = rawHome;

    if (rawHome != stableHome &&
        now - homeChangeTime >= HOME_DEBOUNCE_MS) {

        stableHome = rawHome;
        validHomeTransitions++;

        if (stableHome == HIGH) {
            homeArmed = true;
        }

        if (stableHome == LOW && homeArmed) {
            homeArmed = false;

            if (state == HOMING) {
                homeEvent = true;
                position = 0;
                Serial.println("*** HOME EDGE DETECTED ***");
            }
        }
    }

    // SERIAL
    readSerial();

    // HOMING
    if (state == HOMING && homeEvent) {

        stopMotor();
        homeEvent = false;

        Serial.println("HOME FOUND - MOTOR OFF");
        reportDiagnostics();

        state = HOMING_SETTLE;
    }

    // HOME SETTLE
    if (state == HOMING_SETTLE && settled(now)) {

        Serial.print("Coast past HOME: ");
        Serial.println(position);

        positionValid = true;
        state = IDLE;

        Serial.println("HOME REFERENCE ESTABLISHED.");

        if (pendingMove) {
            pendingMove = false;
            beginMove(targetPosition);
        } else {
            Serial.println("Ready for position command.");
        }
    }

    // MOVE
    if (state == MOVING &&
        position >= cutoffPosition) {

        stopMotor();

        Serial.print("Motor OFF at: ");
        Serial.println(position);

        reportDiagnostics();

        state = MOVE_SETTLE;
    }

    // MOVE SETTLE
    if (state == MOVE_SETTLE && settled(now)) {

        long finalPosition = position;
        long error = finalPosition - targetPosition;

        Serial.println();
        Serial.println("------ POSITION RESULT ------");

        Serial.print("Target: ");
        Serial.println(targetPosition);

        Serial.print("Final position: ");
        Serial.println(finalPosition);

        Serial.print("Error: ");
        if (error > 0) Serial.print("+");
        Serial.println(error);

        Serial.print("Coast distance: ");
        Serial.println(finalPosition - positionAtMotorOff);

        Serial.println("-----------------------------");

        state = IDLE;
        Serial.println("Ready for next command.");
    }

    // PERIODIC DIAGNOSTICS
    if (motorRunning() &&
        now - lastReportTime >= REPORT_INTERVAL_MS) {

        lastReportTime = now;
        reportDiagnostics();
    }

    // TIMEOUT
if (motorRunning() &&
    millis() - motorStartTime >= MOTOR_TIMEOUT_MS) {

    motorError("Motor timeout.");
}
}
