//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_PINSMONITOR_H
#define RELAYCONTROLLER_PINSMONITOR_H

#include "Arduino.h"
#include "Settings.h"


class PinsMonitor {
public:
    PinsMonitor(Settings &settings) : settings(settings) {}
    void setup();
private:
    Settings &settings;
};


#endif //RELAYCONTROLLER_PINSMONITOR_H
