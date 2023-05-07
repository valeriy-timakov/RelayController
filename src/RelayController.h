//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_RELAYCONTROLLER_H
#define RELAYCONTROLLER_RELAYCONTROLLER_H

#include "Arduino.h"
#include "Settings.h"


class RelayController {
public:
    RelayController(Settings &settings) : settings(settings) {}
    void setup();
    void idle();
    void settingsChanged();
    bool isControlTemporaryDisabled(uint8_t relayIdx);
    void setControlTemporaryDisabled(uint8_t relayIdx, bool disabled);
    bool checkRelayMonitoringState(uint8_t relayIdx);
    bool checkControlPinState(uint8_t relayIdx);
    bool getRelayLastState(uint8_t relayIdx);
    void setRelayState(uint8_t relayIdx, bool swithedOn);
private:
    Settings &settings;
};


#endif //RELAYCONTROLLER_RELAYCONTROLLER_H
