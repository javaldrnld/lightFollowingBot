#ifndef LIGHT_DETECTOR_H
#define LIGHT_DETECTOR_H

#include <Arduino.h>

class LightDetector {
    public:
        LightDetector(int pin);
        int read();
    private:
        int _pin;
};

#endif