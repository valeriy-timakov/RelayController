//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_RELAYCONTROLLER_H
#define RELAYCONTROLLER_RELAYCONTROLLER_H

#include "Arduino.h"
#include "Settings.h"

#define SWITCHES_DATA_BUFFER_SIZE 50
#define MAX_SWITCH_LIMIT_COUNT 20


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
    static uint32_t getRemoteTimeStamp() ;
    static void setRemoteTimeStamp(uint32_t remoteTimeStamp);
#ifdef MEM_32KB
    static uint8_t getFixTryCount(uint8_t relayIdx);
    static int32_t getFixLastTryTime(uint8_t relayIdx);
    static uint8_t getSwitchData(uint32_t **data);
    static uint32_t getContactStartWait(uint8_t relayIdx);
#endif
    static uint32_t getRemoteTimeSec();
#ifdef MEM_32KB
    static void clearSwitchCount(uint8_t relayIdx);
#endif

private:
    RelayController() {}
    static void settingsChanged();
};


#endif //RELAYCONTROLLER_RELAYCONTROLLER_H
