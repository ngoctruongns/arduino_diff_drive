#include "config.h"
#include "sensor.h"


void initSensor() {
    pinMode(ULTRA_TRIG_PIN, OUTPUT);
    pinMode(ULTRA_ECHO_PIN, INPUT);
}

// Return distance from HC-SR04 with unit cm

int ultraGetDistance() {
    digitalWrite(ULTRA_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRA_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRA_TRIG_PIN, LOW);

    long duration = pulseIn(ULTRA_ECHO_PIN, HIGH);
    return duration * 0.034 / 2;
}