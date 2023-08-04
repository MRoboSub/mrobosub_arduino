#pragma once

namespace PowerSketch {
    const unsigned long PINS[] = {7};
    void setup() {
        for (const unsigned long pin: PINS) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
        }
    }
}