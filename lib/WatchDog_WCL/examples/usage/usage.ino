SYSTEM_MODE(MANUAL);
#include "WatchDog_WCL.h"

WatchDog wd;

void setup() {
    pinMode(D6, INPUT_PULLUP);
    RGB.control(true);
    if  (System.resetReason() == RESET_REASON_WATCHDOG) {
        RGB.color(255,0,0);
        delay(CONCURRENT_WAIT_FOREVER);
    }  else {
        RGB.color(0,0,255);
    }
    wd.initialize(5000);
    wd.pet();
}

void loop() {
    if (digitalRead(D6) == LOW) {
        wd.pet();
    }
}
