//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_RELAYCONTROLLER_H
#define RELAYCONTROLLER_RELAYCONTROLLER_H

#include "Arduino.h"
#include "Settings.h"


class RelayController {
public:
    static void setup(Settings &settings);
    static void idle();
    static bool isControlTemporaryDisabled(uint8_t relayIdx);
    static void setControlTemporaryDisabled(uint8_t relayIdx, bool disabled);
    static bool checkRelayMonitoringState(uint8_t relayIdx);
    static bool checkControlPinState(uint8_t relayIdx);
    static bool getRelayLastState(uint8_t relayIdx);
    static void setRelayState(uint8_t relayIdx, bool swithedOn);
    static uint16_t getContactReadyWaitDelay() ;
    static void setContactReadyWaitDelay(uint16_t contactReadyWaitDelay);
    static uint16_t getSwitchLimitIntervalSec() ;
    static void setSwitchLimitIntervalSec(uint16_t switchLimitIntervalSec);
    static uint32_t getRemoteTimeStamp() ;
    static void setRemoteTimeStamp(uint32_t remoteTimeStamp);

#ifdef MEM_32KB
    uint8_t getMaxSwitchCount(uint8_t relayIdx) const;
    void setMaxSwitchCount(uint8_t relayIdx, uint8_t maxSwitchCount);
    void clearSwitchCount(uint8_t relayIdx);
#endif

private:
    RelayController() {}
    static void settingsChanged();
};


#endif //RELAYCONTROLLER_RELAYCONTROLLER_H
