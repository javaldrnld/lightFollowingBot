#include <Arduino.h>
#include "LightDetector.h"

LightDetector::LightDetector(int pin) {
    _pin = pin;
    pinMode(_pin, INPUT);
}

int LightDetector::read() {
    return digitalRead(_pin);
}