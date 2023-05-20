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
    uint16_t getContactReadyWaitDelay() const;
    void setContactReadyWaitDelay(uint16_t contactReadyWaitDelay);
    uint16_t getSwitchLimitIntervalSec() const;
    void setSwitchLimitIntervalSec(uint16_t switchLimitIntervalSec);

#ifdef MEM_32KB
    uint8_t getMaxSwitchCount(uint8_t relayIdx) const;
    void setMaxSwitchCount(uint8_t relayIdx, uint8_t maxSwitchCount);
    void clearSwitchCount(uint8_t relayIdx);
#endif

private:
    Settings &settings;
};


#endif //RELAYCONTROLLER_RELAYCONTROLLER_H
