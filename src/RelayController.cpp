//
// Created by valti on 06.05.2023.
//

#include "RelayController.h"
#include "CommunicationProtocol.h"


#define CONTACT_READY_WAIT_DATA_STARTED_BIT 15
#define CONTACT_READY_WAIT_DATA_LAST_STATE_BIT (CONTACT_READY_WAIT_DATA_STARTED_BIT- 1)
#define CONTACT_READY_WAIT_DATA_LAST_CHANGE_LENGTH CONTACT_READY_WAIT_DATA_LAST_STATE_BIT
#define CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK BF_MASK(0, CONTACT_READY_WAIT_DATA_LAST_CHANGE_LENGTH)
#define REQUEST_TIME_STAMP_INTERVAL 10

SettingsPtr settings_;
/*
inline void sendSerialDbg(uint8_t v) { sendSerial(v); }
inline void sendSerialDbg(uint16_t v) { sendSerial(v); }
inline void sendSerialDbg(uint32_t v) { sendSerial(v); }
inline void sendSerialDbg(uint64_t v) { sendSerial(v); }
*/
struct ContactWaitData {
private:
    uint16_t data;
    uint32_t startWaitSec;

    [[nodiscard]] inline bool isWaitFinished() const {
        return (millis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK) - (data & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK)
            >= (settings_.getStateFixSettings().getContactReadyWaitDelayMillis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK);
    }
    inline void startWait(bool pinSet) {
        startWaitSec = RelayController::getRemoteTimeSec();
        data = (1 << CONTACT_READY_WAIT_DATA_STARTED_BIT) | (millis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK);
        setBit(data, CONTACT_READY_WAIT_DATA_LAST_STATE_BIT, pinSet);
    }
    inline void stopWait() {
        data = 0;
        startWaitSec = 0;
    }
    inline void update(bool value) {
        bool lastStateOn = CHECK_BIT(data, CONTACT_READY_WAIT_DATA_LAST_STATE_BIT);
        if (lastStateOn != value) {
            data = (1 << CONTACT_READY_WAIT_DATA_STARTED_BIT) | (millis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK);
            setBit(data, CONTACT_READY_WAIT_DATA_LAST_STATE_BIT, value);
        }
    }
public:
    ContactWaitData() : data(0), startWaitSec(0) {}
    [[nodiscard]] inline bool isWaitStarted() const {
        return CHECK_BIT(data, CONTACT_READY_WAIT_DATA_STARTED_BIT);
    }
    inline bool checkReady(bool ctrlPinSet) {
        if (!isWaitStarted()) {
            startWait(ctrlPinSet);
            return false;
        } else if (isWaitFinished()) {
            stopWait();
            return true;
        } else {
            update(ctrlPinSet);
            return false;
        }
    }
    [[nodiscard]] inline uint32_t getStartWaitSec() const {
        return startWaitSec;
    }
};


#ifdef MEM_32KB
#define SWITCH_LIMIT_DATA_LENGTH 4
#define SWITCH_LIMIT_DATA_MASK BF_MASK(0, SWITCH_LIMIT_DATA_LENGTH)
#define SWITCH_LIMIT_MONITOR_DATA_CAPACITY (1 << SWITCH_LIMIT_DATA_LENGTH)
#define SWITCH_LIMIT_DATA_COUNT_PER_BYTE (8 / SWITCH_LIMIT_DATA_LENGTH)
#define SWITCH_LIMITER_DATA_BYTES_COUNT (MAX_SWITCH_LIMIT_COUNT / SWITCH_LIMIT_DATA_COUNT_PER_BYTE)
#define MONITORING_INTERVAL_PER_CONTROL_INTERVAL_RATIO 2
#define SWITCH_LIMIT_CONTROL_DATA_CAPACITY (SWITCH_LIMIT_MONITOR_DATA_CAPACITY / MONITORING_INTERVAL_PER_CONTROL_INTERVAL_RATIO)

struct SwitchLimiter {
    uint8_t data[SWITCH_LIMITER_DATA_BYTES_COUNT];
    uint8_t count = 0;

    void update() {
        uint16_t switchLimitIntervalSec = settings_.getSwitchCountingSettingsRef().getSwitchLimitIntervalSec();
        auto monitoringInterval = (uint32_t) MILLIS_PER_SECOND * switchLimitIntervalSec * MONITORING_INTERVAL_PER_CONTROL_INTERVAL_RATIO;
        auto stamp = (uint8_t) ( ( millis() % monitoringInterval ) * SWITCH_LIMIT_MONITOR_DATA_CAPACITY / monitoringInterval);
        auto controlInterval = (uint32_t) MILLIS_PER_SECOND * switchLimitIntervalSec;
        auto controlStamp = (uint8_t) ( ( millis() % controlInterval ) * SWITCH_LIMIT_CONTROL_DATA_CAPACITY / controlInterval);

        uint8_t outdatedCount = 0;
        uint8_t blocksCount = (count + SWITCH_LIMIT_DATA_COUNT_PER_BYTE - 1) / SWITCH_LIMIT_DATA_COUNT_PER_BYTE;
        uint8_t maxOutdatedValue;
        if (stamp - controlStamp == 0) {
            maxOutdatedValue = controlStamp;
        } else {
            maxOutdatedValue = controlStamp + SWITCH_LIMIT_CONTROL_DATA_CAPACITY;
        }
        for (uint8_t i = 0; i < blocksCount; i++) {
            if ((data[i / SWITCH_LIMIT_DATA_LENGTH] & SWITCH_LIMIT_DATA_MASK) <= maxOutdatedValue) {
                outdatedCount++;
                if (i + 1 < count && (data[i / SWITCH_LIMIT_DATA_LENGTH] >> SWITCH_LIMIT_DATA_LENGTH) <= maxOutdatedValue) {
                    outdatedCount++;
                } else {
                    break;
                }
            } else {
                break;
            }
        }
        count -= outdatedCount;
        if (outdatedCount % SWITCH_LIMIT_DATA_COUNT_PER_BYTE == 0) {
            for (uint8_t i = 0; i < blocksCount; i++) {
                uint8_t earliestActualBlock = i + outdatedCount / SWITCH_LIMIT_DATA_COUNT_PER_BYTE;
                if (earliestActualBlock < blocksCount) {
                    data[i] = data[earliestActualBlock];
                }
            }
        } else {
            for (uint8_t i = 0; i < blocksCount; i++) {
                uint8_t earliestActualBlock = i + outdatedCount / SWITCH_LIMIT_DATA_COUNT_PER_BYTE;
                if (earliestActualBlock < blocksCount) {
                    data[i] = data[earliestActualBlock] >> SWITCH_LIMIT_DATA_LENGTH;
                    if (earliestActualBlock + 1 < blocksCount) {
                        data[i] |= data[earliestActualBlock + 1] << SWITCH_LIMIT_DATA_LENGTH;
                    }
                }
            }
        }
    }

    bool tryAdd() {
        uint8_t switchMaxCount = settings_.getSwitchCountingSettingsRef().getMaxSwitchCount();
        if (switchMaxCount == 0) {
            return true;
        }
        if (switchMaxCount > MAX_SWITCH_LIMIT_COUNT) {
            switchMaxCount = MAX_SWITCH_LIMIT_COUNT;
        }
        if (count >= switchMaxCount) {
            return false;
        }
        uint16_t switchLimitIntervalSec = settings_.getSwitchCountingSettingsRef().getSwitchLimitIntervalSec();
        auto monitoringInterval = (uint32_t) MILLIS_PER_SECOND * switchLimitIntervalSec * MONITORING_INTERVAL_PER_CONTROL_INTERVAL_RATIO;
        auto stamp = (uint8_t)( ( (millis() % monitoringInterval) ) * SWITCH_LIMIT_MONITOR_DATA_CAPACITY / monitoringInterval);
        if (count < MAX_SWITCH_LIMIT_COUNT - 1) {
            if (count % SWITCH_LIMIT_DATA_COUNT_PER_BYTE == 1) {
                data[count / SWITCH_LIMIT_DATA_LENGTH] |= stamp << SWITCH_LIMIT_DATA_LENGTH;
            } else {
                data[count / SWITCH_LIMIT_DATA_LENGTH] = stamp;
            }
            count++;
            return true;
        } else {
            return false;
        }
    }

    void clear() {
        count = 0;
    }
};
#endif

ContactWaitData lastChangeWaitDatas[MAX_RELAYS_COUNT];
#ifdef MEM_32KB
SwitchLimiter switchLimiters[MAX_RELAYS_COUNT];
#endif
bool lastInterruptPinHigh = false;
uint16_t lastControlState = 0;
uint16_t lastRelayState = 0;
uint16_t temporaryDisabledControls = 0;
uint32_t startLocalTimeSec = 0;
uint32_t remoteTimeStamp = 0;
uint32_t stateSwitchDatas[SWITCHES_DATA_BUFFER_SIZE];
uint8_t stateSwitchCount = 0;
int32_t stateFixTimes[MAX_RELAYS_COUNT];
uint8_t stateFixCount[MAX_RELAYS_COUNT];
uint32_t lastTimeStampRequsetTime = 0;
uint16_t lastMonitoringState = 0;


void switchRelayState(const RelaySettings &settings, uint8_t i);

bool getLastControlState(uint8_t relayIdx) {
    return CHECK_BIT(lastControlState, relayIdx);
}

void setLastControlState(uint8_t relayIdx, bool switchedOn) {
    setBit(lastControlState, relayIdx, switchedOn);
}

bool getLastRelayState(uint8_t relayIdx) {
    return CHECK_BIT(lastRelayState, relayIdx);
}

void setLastRelayState(uint8_t relayIdx, bool switchedOn) {
    setBit(lastRelayState, relayIdx, switchedOn);
}

bool isControlTemporaryDisabled_(uint8_t relayIdx) {
    return CHECK_BIT(temporaryDisabledControls, relayIdx);
}

bool checkPinState(const PinSettings &pinSettings) {
    if (pinSettings.isEnabled() && pinSettings.isAllowedPin()) {
        bool high = digitalRead(pinSettings.getPin()) == HIGH;
        return pinSettings.isInversed() == !high;
    }
    return false;
}

uint32_t getLocalTimeSec() {
    return millis() / MILLIS_PER_SECOND - startLocalTimeSec;
}

inline void writePinStateForse(const PinSettings &pinSettings, bool switchedOn) {
    digitalWrite(pinSettings.getPin(), pinSettings.isInversed() != switchedOn ? HIGH : LOW);
}

void setRelayState_(const RelaySettings &relaySettings, bool switchedOn, uint8_t relayIdx, bool internal) {
    const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
    if (setPinSettings.isAllowedPin() && setPinSettings.isEnabled()) {
        writePinStateForse(setPinSettings, switchedOn);
        setLastRelayState(relayIdx, switchedOn);
        uint32_t switchTimeData = RelayController::getRemoteTimeSec();
        switchTimeData = (switchTimeData & 0x03ffffff) | (((uint32_t)relayIdx) << 28);
        if (switchedOn) {
            switchTimeData |= 1l << 27;
        }
        if (internal) {
            switchTimeData |= 1l << 26;
        }
        stateSwitchDatas[stateSwitchCount++] = switchTimeData;
        stateFixTimes[relayIdx] = getLocalTimeSec();
        stateFixCount[relayIdx] = 0;
        sendStartSignal(IDC_RELAY_STATE_CHANGED);
        sendSerial(switchTimeData);

    }
}

void checkAndFixRelayStates() {
    for (uint8_t relayIdx = 0; relayIdx < settings_.getRelaysCount(); relayIdx++) {
        const RelaySettings &relaySettings = settings_.getRelaySettingsRef(relayIdx);
        const PinSettings &monitoringPinSettings = relaySettings.getMonitorPinSettings();
        const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
        bool switchedOnByMonitoring = checkPinState(monitoringPinSettings);
        bool lastMonitoringSwithedOn = CHECK_BIT(lastMonitoringState, relayIdx);
        if (switchedOnByMonitoring != lastMonitoringSwithedOn) {
            uint8_t data = relayIdx & 0xf;
            if (switchedOnByMonitoring) {
                data |= 0x10;
            }
            sendStartSignal(IDC_MONITORING_STATE_CHANGED);
            sendSerial(data);
            sendSerial(RelayController::getRemoteTimeSec());
            setBit(lastMonitoringState, relayIdx, switchedOnByMonitoring);
        }
        bool switchedOn = getLastRelayState(relayIdx);
        if (
            switchedOnByMonitoring != switchedOn && monitoringPinSettings.isEnabled()
            && setPinSettings.isEnabled()
            && setPinSettings.isAllowedPin()
            && stateFixCount[relayIdx] < settings_.getStateFixSettings().getMaxCount()
            && getLocalTimeSec() - stateFixTimes[relayIdx] >= settings_.getStateFixSettings().getMinWaitDelaySec()
        ) {
            writePinStateForse(setPinSettings, !switchedOn);
            delay(settings_.getStateFixSettings().getDelayMillis());
            writePinStateForse(setPinSettings, switchedOn);
            stateFixTimes[relayIdx] = getLocalTimeSec();
            stateFixCount[relayIdx]++;
        }
    }
}

void checkAndProcessChanges() {
    for (uint8_t i = 0; i < settings_.getRelaysCount(); i++) {
        const RelaySettings &relaySettings = settings_.getRelaySettingsRef(i);
        const PinSettings &ctrlPinSettings = relaySettings.getControlPinSettings();
        const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
        if (ctrlPinSettings.isEnabled() && setPinSettings.isEnabled() && !isControlTemporaryDisabled_(i)) {
            bool ctrlPinSet = checkPinState(ctrlPinSettings);
            bool lastSwithedOn = getLastControlState(i);
            auto &lastWaitData = lastChangeWaitDatas[i];
            if (
                (lastSwithedOn != ctrlPinSet || lastWaitData.isWaitStarted()) &&
                lastWaitData.checkReady(ctrlPinSet)
                #ifdef MEM_32KB
                && switchLimiters[i].tryAdd()
                #endif
            ) {
                sendStartSignal(IDC_CONTROL_STATE_CHANGED);
                sendSerial((uint8_t)((ctrlPinSet ? 0x10 : 0x00) | (i & 0xf)));
                if (relaySettings.isControlPinSwitchByPush()) {
                    if (ctrlPinSet) {
                        switchRelayState(relaySettings, i);
                    }
                } else {
                    setRelayState_(relaySettings, ctrlPinSet, i, true);
                }
                setLastControlState(i, ctrlPinSet);
            }
        }
    }
}

void onControlPinChange() {
    bool newInterruptPinHigh = digitalRead(settings_.getControlInterruptPin()) == HIGH;
    if (newInterruptPinHigh != lastInterruptPinHigh) {
        checkAndProcessChanges();
        lastInterruptPinHigh = newInterruptPinHigh;
    }
}

void switchRelayState(const RelaySettings &settings, uint8_t i) {
    bool switchedOn = !getLastRelayState(i);
    setRelayState_(settings, switchedOn, i, true);
}

void RelayController::settingsChanged() {
    uint8_t relaysCount = settings_.getRelaysCount();
    for (uint8_t i = 0; i < relaysCount; i++) {
        const RelaySettings &relaySettings = settings_.getRelaySettingsRef(i);
        const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
        if (setPinSettings.isEnabled()) {
            if (setPinSettings.isAllowedPin()) {
                pinMode(setPinSettings.getPin(), OUTPUT);
            }
        }
        const PinSettings &monitorPinSettings = relaySettings.getMonitorPinSettings();
        if (monitorPinSettings.isEnabled()) {
            if (monitorPinSettings.isAllowedPin()) {
                pinMode(monitorPinSettings.getPin(), monitorPinSettings.isInversed() ? INPUT_PULLUP : INPUT);
            }
        }
        const PinSettings &controlPinSettings = relaySettings.getControlPinSettings();
        if (controlPinSettings.isEnabled()) {
            if (controlPinSettings.isAllowedPin()) {
                pinMode(controlPinSettings.getPin(), controlPinSettings.isInversed() ? INPUT_PULLUP : INPUT);
            }
        }
    }
    attachInterrupt(digitalPinToInterrupt(settings_.getControlInterruptPin()), onControlPinChange, CHANGE);
}



void RelayController::setup(Settings &settings) {
    settings_ = settings.getRelaysSettingsPtr();
    if (!settings.isReady()) {
        settings.load();
    }
    settingsChanged();
    settings.setOnSettingsChanged(settingsChanged);
    for (auto& lastChangeStartTime : lastChangeWaitDatas) {
        lastChangeStartTime = ContactWaitData();
    }
    startLocalTimeSec = 0;
    remoteTimeStamp = 0;
    for (uint8_t i = 0; i < MAX_RELAYS_COUNT; i++) {
        stateSwitchDatas[i] = 0;
        stateFixTimes[i] = 0;
        stateFixCount[i] = 0;
    }
    stateSwitchCount = 0;
}

void RelayController::idle() {
    if (startLocalTimeSec == 0 && (lastTimeStampRequsetTime - getLocalTimeSec() > REQUEST_TIME_STAMP_INTERVAL)) {
        sendStartSignal(IDC_GET_TIME_STAMP);
        lastTimeStampRequsetTime = getLocalTimeSec();
    }
#ifdef MEM_32KB
    for (uint8_t i = 0; i < settings_.getRelaysCount(); i++) {
        switchLimiters[i].update();
    }
#endif
    checkAndProcessChanges();
    checkAndFixRelayStates();
}

bool RelayController::isControlTemporaryDisabled(uint8_t relayIdx) {
    return isControlTemporaryDisabled_(relayIdx);
}

void RelayController::setControlTemporaryDisabled(uint8_t relayIdx, bool disabled) {
    setBit(temporaryDisabledControls, relayIdx, disabled);
}

bool RelayController::checkRelayMonitoringState(uint8_t relayIdx) {
    if (relayIdx >= settings_.getRelaysCount()) {
        return false;
    }
    return checkPinState(settings_.getRelaySettingsRef(relayIdx).getMonitorPinSettings());
}

bool RelayController::checkControlPinState(uint8_t relayIdx) {
    if (relayIdx >= settings_.getRelaysCount()) {
        return false;
    }
    return checkPinState(settings_.getRelaySettingsRef(relayIdx).getControlPinSettings());
}

bool RelayController::getRelayLastState(uint8_t relayIdx) {
    return getLastRelayState(relayIdx);
}

void RelayController::setRelayState(uint8_t relayIdx, bool switchedOn) {
    if (relayIdx >= settings_.getRelaysCount()) {
        return;
    }
    setRelayState_(settings_.getRelaySettingsRef(relayIdx), switchedOn, relayIdx, false);
}

uint32_t RelayController::getRemoteTimeStamp() {
    return remoteTimeStamp;
}

void RelayController::setRemoteTimeStamp(uint32_t value) {
    remoteTimeStamp = value;
    uint32_t newStartLocalTimeSec = millis() / MILLIS_PER_SECOND;
    uint32_t diff = newStartLocalTimeSec - startLocalTimeSec;
    for (uint8_t i = 0; i < settings_.getRelaysCount(); i++) {
        stateFixTimes[i] -= diff;
    }
    startLocalTimeSec = newStartLocalTimeSec;


}

uint8_t RelayController::getFixTryCount(uint8_t relayIdx) {
    if (relayIdx >= settings_.getRelaysCount()) {
        return 0;
    }
    return stateFixCount[relayIdx];
}

int32_t RelayController::getFixLastTryTime(uint8_t relayIdx) {
    if (relayIdx >= settings_.getRelaysCount()) {
        return 0;
    }
    return stateFixTimes[relayIdx];
}

uint8_t RelayController::getSwitchData(uint32_t **data) {
    *data = (uint32_t*)stateSwitchDatas;
    uint8_t stateSwitchCount_ = stateSwitchCount;
    stateSwitchCount = 0;
    return stateSwitchCount_;
}

uint32_t RelayController::getContactStartWait(uint8_t relayIdx) {
    return lastChangeWaitDatas[relayIdx].getStartWaitSec();
}

uint32_t RelayController::getRemoteTimeSec() {
    return getLocalTimeSec() + remoteTimeStamp;
}

#ifdef MEM_32KB
void RelayController::clearSwitchCount(uint8_t relayIdx) {
    switchLimiters[relayIdx].clear();
}
#endif

