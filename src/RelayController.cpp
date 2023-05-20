//
// Created by valti on 06.05.2023.
//

#include "RelayController.h"



#define CONTACT_READY_WAIT_DATA_STARTED_BIT 15
#define CONTACT_READY_WAIT_DATA_LAST_STATE_BIT 14
#define CONTACT_READY_WAIT_DATA_LAST_CHANGE_LENGTH CONTACT_READY_WAIT_DATA_LAST_STATE_BIT
#define CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK BF_MASK(0, CONTACT_READY_WAIT_DATA_LAST_CHANGE_LENGTH)
#define DEFAULT_CONTACT_READY_WAIT_DELAY 50
uint16_t contactReadyWaitDelay = DEFAULT_CONTACT_READY_WAIT_DELAY;
uint16_t switchLimitIntervalSec = 600;

struct ContactWaitData {
    uint16_t data = 0;
    bool isWaitStarted() const {
        return CHECK_BIT(data, CONTACT_READY_WAIT_DATA_STARTED_BIT);
    }
    bool isWaitFinished() const {
        return (millis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK) - (data & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK) >= contactReadyWaitDelay;
    }
    void startWait() {
        data = millis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK;
        setBit(data, CONTACT_READY_WAIT_DATA_STARTED_BIT, true);
    }
    void stopWait() {
        data = 0;
    }
    void update(bool value) {
        bool lastStateOn = CHECK_BIT(data, CONTACT_READY_WAIT_DATA_LAST_STATE_BIT);
        if (lastStateOn != value) {
            data = (data & ~CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK) | (millis() & CONTACT_READY_WAIT_DATA_LAST_CHANGE_MASK);
            setBit(data, CONTACT_READY_WAIT_DATA_LAST_STATE_BIT, value);
        }
    }
    bool checkReady(bool ctrlPinSet) {
        if (!isWaitStarted()) {
            startWait();
            return false;
        } else if (isWaitFinished()) {
            stopWait();
            return true;
        } else {
            update(ctrlPinSet);
            return false;
        }
    }
};


#ifdef MEM_32KB
#define MAX_SWITCH_LIMIT_COUNT 20
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
    uint8_t maxCount = 0;// maxCount == 0 means no limit

    void update() {
        auto monitoringInterval = (uint32_t) 1000 * switchLimitIntervalSec * MONITORING_INTERVAL_PER_CONTROL_INTERVAL_RATIO;
        auto stamp = (uint8_t) ( ( millis() % monitoringInterval ) * SWITCH_LIMIT_MONITOR_DATA_CAPACITY / monitoringInterval);
        auto controlInterval = (uint32_t) 1000 * switchLimitIntervalSec;
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
        if (maxCount == 0) {
            return true;
        }
        if (count >= maxCount) {
            return false;
        }
        auto monitoringInterval = (uint32_t) 1000 * switchLimitIntervalSec * MONITORING_INTERVAL_PER_CONTROL_INTERVAL_RATIO;
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

    bool setMaxCount(uint8_t _maxCount) {
        if (_maxCount > MAX_SWITCH_LIMIT_COUNT) {
            return false;
        }
        maxCount = _maxCount;
        return true;
    }

    uint8_t getMaxCount() {
        return maxCount;
    }
};
#endif

ContactWaitData lastChangeWaitDatas[MAX_RELAYS_COUNT];
#ifdef MEM_32KB
SwitchLimiter switchLimiters[MAX_RELAYS_COUNT];
#endif
bool lastInterruptPinHigh = false;
SettingsPtr _settings;
uint16_t _lastControlState = 0;
uint16_t _tmpLastControlState = 0;
uint16_t _lastRelayState = 0;
uint16_t _temporaryDisabledControls = 0;


void switchRelayState(const RelaySettings &settings, uint8_t i);

bool getLastControlState(uint8_t relayIdx) {
    return CHECK_BIT(_lastControlState, relayIdx);
}

void setLastControlState(uint8_t relayIdx, bool swithedOn) {
    setBit(_lastControlState, relayIdx, swithedOn);
}

bool getTmpLastControlState(uint8_t relayIdx) {
    return CHECK_BIT(_tmpLastControlState, relayIdx);
}

void setTmpLastControlState(uint8_t relayIdx, bool swithedOn) {
    setBit(_tmpLastControlState, relayIdx, swithedOn);
}

bool getLastRelayState(uint8_t relayIdx) {
    return CHECK_BIT(_lastRelayState, relayIdx);
}

void setLastRelayState(uint8_t relayIdx, bool swithedOn) {
    setBit(_lastRelayState, relayIdx, swithedOn);
}

bool _isControlTemporaryDisabled(uint8_t relayIdx) {
    return CHECK_BIT(_temporaryDisabledControls, relayIdx);
}

bool RelayController::isControlTemporaryDisabled(uint8_t relayIdx) {
    return _isControlTemporaryDisabled(relayIdx);
}

void RelayController::setControlTemporaryDisabled(uint8_t relayIdx, bool disabled) {
    setBit(_temporaryDisabledControls, relayIdx, disabled);
}


bool checkPinState(const PinSettings &pinSettings) {
    if (pinSettings.isEnabled() && pinSettings.isAllowedPin()) {
        bool high = digitalRead(pinSettings.getPin()) == HIGH;
        return pinSettings.isInversed() == !high;
    }
    return false;
}

void setRelayState_(const RelaySettings &relaySettings, bool swithedOn, uint8_t relayIdx) {
    const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
    if (setPinSettings.isAllowedPin()) {
        if (setPinSettings.isEnabled()) {
            Serial.write(0xfd);
            Serial.write(relayIdx);
            Serial.write(setPinSettings.getPin());
            Serial.write(setPinSettings.isInversed()  ? 0x01 : 0x00);
            Serial.write(swithedOn ? 0x01 : 0x00);
            digitalWrite(setPinSettings.getPin(), setPinSettings.isInversed() == !swithedOn ? HIGH : LOW);
        }
        setLastRelayState(relayIdx, swithedOn);
    }
    Serial.write(0xfa);
}

void checkAndProcessChanges() {
    for (uint8_t i = 0; i < _settings.getRelaysCount(); i++) {
        const RelaySettings &relaySettings = _settings.getRelaySettingsRef(i);
        const PinSettings &ctrlPinSettings = relaySettings.getControlPinSettings();
        const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
        if (ctrlPinSettings.isEnabled() && setPinSettings.isEnabled() && !_isControlTemporaryDisabled(i)) {
            bool ctrlPinSet = checkPinState(ctrlPinSettings);
            bool lastSwithedOn = getLastControlState(i);
            auto lastWaitData = lastChangeWaitDatas[i];
            if (
                (lastSwithedOn != ctrlPinSet || lastWaitData.isWaitStarted()) &&
                lastWaitData.checkReady(ctrlPinSet)
                #ifdef MEM_32KB
                && switchLimiters[i].tryAdd()
                #endif
            ) {
                Serial.write(0xfe);//relay change
                if (relaySettings.isControlPinSwitchByPush() && ctrlPinSet) {
                    switchRelayState(relaySettings, i);
                } else {
                    setRelayState_(relaySettings, ctrlPinSet, i);
                }
                setLastControlState(i, ctrlPinSet);
            }
        }
    }
}

void onControlPinChange() {
    bool newInterruptPinHigh = digitalRead(_settings.getControlInterruptPin()) == HIGH;
    if (newInterruptPinHigh != lastInterruptPinHigh) {
        Serial.write(0xff);//interrupt enter
        checkAndProcessChanges();
        lastInterruptPinHigh = newInterruptPinHigh;
    }
}

void switchRelayState(const RelaySettings &settings, uint8_t i) {
    bool swithedOn = !getLastRelayState(i);
    setRelayState_(settings, swithedOn, i);
}

void RelayController::settingsChanged() {
    _settings = settings.getRelaysSettingsPtr();
    uint8_t relaysCount = settings.getRelaysCount();
    for (uint8_t i = 0; i < relaysCount; i++) {
        const RelaySettings &relaySettings = settings.getRelaySettingsRef(i);
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
    attachInterrupt(digitalPinToInterrupt(settings.getControlInterruptPin()), onControlPinChange, CHANGE);
}



void RelayController::setup() {
    if (!settings.isReady()) {
        settings.load();
    }
    settingsChanged();
    for (auto& lastChangeStartTime : lastChangeWaitDatas) {
        lastChangeStartTime = ContactWaitData();
    }
}

void RelayController::idle() {
#ifdef MEM_32KB
    for (uint8_t i = 0; i < settings.getRelaysCount(); i++) {
        switchLimiters[i].update();
    }
#endif
    checkAndProcessChanges();
}

bool RelayController::checkRelayMonitoringState(uint8_t relayIdx) {
    if (relayIdx >= settings.getRelaysCount()) {
        return false;
    }
    return checkPinState(settings.getRelaySettingsRef(relayIdx).getMonitorPinSettings());
}

bool RelayController::checkControlPinState(uint8_t relayIdx) {
    if (relayIdx >= settings.getRelaysCount()) {
        return false;
    }
    return checkPinState(settings.getRelaySettingsRef(relayIdx).getControlPinSettings());
}

bool RelayController::getRelayLastState(uint8_t relayIdx) {
    return getLastRelayState(relayIdx);
}

void RelayController::setRelayState(uint8_t relayIdx, bool swithedOn) {
    if (relayIdx >= settings.getRelaysCount()) {
        return;
    }
    setRelayState_(settings.getRelaySettingsRef(relayIdx), swithedOn, relayIdx);
}

uint16_t RelayController::getContactReadyWaitDelay() const {
    return contactReadyWaitDelay;
}

void RelayController::setContactReadyWaitDelay(uint16_t value) {
    contactReadyWaitDelay = value;
}

uint16_t RelayController::getSwitchLimitIntervalSec() const {
    return switchLimitIntervalSec;
}

void RelayController::setSwitchLimitIntervalSec(uint16_t value) {
    switchLimitIntervalSec = value;
}

#ifdef MEM_32KB
uint8_t RelayController::getMaxSwitchCount(uint8_t relayIdx) const {
    return switchLimiters[relayIdx].getMaxCount();
}

void RelayController::setMaxSwitchCount(uint8_t relayIdx, uint8_t value) {
    switchLimiters[relayIdx].setMaxCount(value);
}

void RelayController::clearSwitchCount(uint8_t relayIdx) {
    switchLimiters[relayIdx].clear();
}
#endif

