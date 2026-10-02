#include <Arduino.h>

#define BUTTON_PIN         14
#define MOTOR_FWD_PIN      16
#define MOTOR_REV_PIN      17
#define CAM_HOME_PIN       13
#define GEAR_ROTATION_PIN  33

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(MOTOR_FWD_PIN, OUTPUT);
    pinMode(MOTOR_REV_PIN, OUTPUT);

    pinMode(CAM_HOME_PIN, INPUT_PULLUP);
    pinMode(GEAR_ROTATION_PIN, INPUT_PULLUP);

    // Motor OFF
    digitalWrite(MOTOR_FWD_PIN, LOW);
    digitalWrite(MOTOR_REV_PIN, LOW);

    Serial.println();
    Serial.println("Furby motor test ready.");
    Serial.println("Press button for one 100 ms motor pulse.");
}

void loop() {
    static bool lastButton = HIGH;

    bool button = digitalRead(BUTTON_PIN);

    if (lastButton == HIGH && button == LOW) {

        Serial.println();
        Serial.println("MOTOR PULSE");

        // Forward only
        digitalWrite(MOTOR_REV_PIN, LOW);
        digitalWrite(MOTOR_FWD_PIN, HIGH);

        unsigned long start = millis();

        while (millis() - start < 100) {

            if (digitalRead(CAM_HOME_PIN) == LOW) {
                Serial.println("CAM HOME active");
            }

            if (digitalRead(GEAR_ROTATION_PIN) == LOW) {
                Serial.println("GEAR ROTATION active");
            }
        }

        // Motor OFF
        digitalWrite(MOTOR_FWD_PIN, LOW);

        Serial.println("Motor stopped.");
    }

    lastButton = button;
    delay(10);
}