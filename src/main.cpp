#include <Arduino.h>

// ------------------------------------------------------------
// PIN ASSIGNMENTS
// ------------------------------------------------------------

#define BUTTON_PIN         14

#define MOTOR_FWD_PIN      16
#define MOTOR_REV_PIN      17
#define POWER_DOWN_PIN     32

#define CAM_HOME_PIN       13
#define GEAR_ROTATION_PIN  33


// ------------------------------------------------------------
// MOTOR
// ------------------------------------------------------------

const unsigned long MOTOR_PULSE_MS = 100;

bool motorRunning = false;
unsigned long motorStartTime = 0;


// ------------------------------------------------------------
// POSITION TRACKING
// ------------------------------------------------------------

bool lastGearState;
bool haveHome = false;
bool homeArmed = true;

unsigned long totalTicks = 0;
unsigned int positionTicks = 0;

// CAM HOME is active for ~5 ticks.
// Don't allow another HOME until we're well beyond it.
const unsigned int HOME_REARM_TICKS = 15;


// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------

void setup() {

    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(CAM_HOME_PIN, INPUT_PULLUP);
    pinMode(GEAR_ROTATION_PIN, INPUT_PULLUP);

    pinMode(MOTOR_FWD_PIN, OUTPUT);
    pinMode(MOTOR_REV_PIN, OUTPUT);
    pinMode(POWER_DOWN_PIN, OUTPUT);

    digitalWrite(MOTOR_FWD_PIN, LOW);
    digitalWrite(MOTOR_REV_PIN, LOW);

    digitalWrite(POWER_DOWN_PIN, HIGH);

    lastGearState = digitalRead(GEAR_ROTATION_PIN);

    Serial.println();
    Serial.println("--------------------------------");
    Serial.println("FURBY CONTINUOUS POSITION TRACKER");
    Serial.println("--------------------------------");
    Serial.println("Press button for 100 ms motor pulse.");
}


// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------

void loop() {

    // ========================================================
    // 1. READ GEAR ROTATION
    // ========================================================

    bool gearState = digitalRead(GEAR_ROTATION_PIN);

    // Count HIGH -> LOW only
    if (lastGearState == HIGH &&
        gearState == LOW) {

        totalTicks++;

        if (haveHome) {
            positionTicks++;
        }
    }

    lastGearState = gearState;


    // ========================================================
    // 2. RE-ARM CAM HOME
    // ========================================================

    if (!homeArmed &&
        positionTicks >= HOME_REARM_TICKS) {

        homeArmed = true;
    }


    // ========================================================
    // 3. READ CAM HOME
    // ========================================================

    bool camState = digitalRead(CAM_HOME_PIN);

    if (camState == LOW && homeArmed) {

        Serial.println();

        if (!haveHome) {

            Serial.println("*** FIRST HOME ***");
            Serial.println("Position synchronized.");

            haveHome = true;
        }

        else {

            Serial.print("*** HOME -- cycle length: ");
            Serial.print(positionTicks);
            Serial.println(" ticks ***");
        }

        // HOME becomes position zero
        positionTicks = 0;

        // Ignore chatter and the rest of the HOME region
        homeArmed = false;
    }


    // ========================================================
    // 4. BUTTON
    // ========================================================

    static bool lastButton = HIGH;

    bool button = digitalRead(BUTTON_PIN);

    if (lastButton == HIGH &&
        button == LOW &&
        !motorRunning) {

        Serial.println();
        Serial.println("MOTOR ON");

        digitalWrite(MOTOR_REV_PIN, LOW);
        digitalWrite(MOTOR_FWD_PIN, HIGH);

        motorStartTime = millis();
        motorRunning = true;
    }

    lastButton = button;


    // ========================================================
    // 5. MOTOR TIMER
    // ========================================================

    if (motorRunning &&
        millis() - motorStartTime >= MOTOR_PULSE_MS) {

        digitalWrite(MOTOR_FWD_PIN, LOW);

        motorRunning = false;

        Serial.println("MOTOR OFF");

        if (haveHome) {

            Serial.print("Position at motor OFF: ");
            Serial.println(positionTicks);
        }
    }
}