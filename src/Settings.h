//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_SETTINGS_H
#define RELAYCONTROLLER_SETTINGS_H

#include "Arduino.h"
#include "utils.h"

#define DEFAULT_STATE_FIX_DELAY 100
#define DEFAULT_STATE_FIX_MAX_COUNT 3
#define DEFAULT_STATE_FIX_WAIT_DELAY 10
#define DEFAULT_CONTACT_READY_WAIT_DELAY 50
#define DEFAULT_SWITCH_LIMIT_INTERVAL_SEC 0
#define DEFAULT_SWITCH_MAX_COUNT 0

struct StateFixSettings {
private:
    uint16_t delayMillis;
    uint8_t maxCount;
    uint8_t minWaitDelaySec;
    uint16_t contactReadyWaitDelayMillis;
public:
    StateFixSettings(uint16_t delayMillis, uint8_t maxCount, uint8_t minWaitDelaySec, uint16_t contactReadyWaitDelay) :
            delayMillis(delayMillis), maxCount(maxCount), minWaitDelaySec(minWaitDelaySec),
            contactReadyWaitDelayMillis(contactReadyWaitDelay) {}
    StateFixSettings() :
            delayMillis(DEFAULT_STATE_FIX_DELAY), maxCount(DEFAULT_STATE_FIX_MAX_COUNT),
            minWaitDelaySec(DEFAULT_STATE_FIX_WAIT_DELAY), contactReadyWaitDelayMillis(DEFAULT_CONTACT_READY_WAIT_DELAY) {}
    StateFixSettings& operator=(StateFixSettings const& src) {
        if (this != &src) {
            delayMillis = src.delayMillis;
            maxCount = src.maxCount;
            minWaitDelaySec = src.minWaitDelaySec;
            contactReadyWaitDelayMillis = src.contactReadyWaitDelayMillis;
        }
        return *this;
    }

    [[nodiscard]] inline uint16_t getDelayMillis() const {
        return delayMillis;
    }

    [[nodiscard]] inline uint8_t getMaxCount() const {
        return maxCount;
    }

    [[nodiscard]] inline uint8_t getMinWaitDelaySec() const {
        return minWaitDelaySec;
    }

    [[nodiscard]] inline uint16_t getContactReadyWaitDelayMillis() const {
        return contactReadyWaitDelayMillis;
    }
};

#ifdef MEM_32KB
struct SwitchCountingSettings {
private:
    uint16_t switchLimitIntervalSec;
    uint8_t switchMaxCount;// switchMaxCount == 0 means no limit
public:
    SwitchCountingSettings(uint16_t switchLimitIntervalSec, uint8_t switchMaxCount) :
        switchLimitIntervalSec(switchLimitIntervalSec), switchMaxCount(switchMaxCount) {}
    SwitchCountingSettings() :
            switchLimitIntervalSec(DEFAULT_SWITCH_LIMIT_INTERVAL_SEC), switchMaxCount(DEFAULT_SWITCH_MAX_COUNT) {}
    SwitchCountingSettings& operator=(SwitchCountingSettings const& src) {
        if (this != &src) {
            switchLimitIntervalSec = src.switchLimitIntervalSec;
            switchLimitIntervalSec = src.switchLimitIntervalSec;
        }
        return *this;
    }
    [[nodiscard]] inline uint8_t getMaxSwitchCount() const {
        return switchMaxCount;
    }
    [[nodiscard]] inline uint16_t getSwitchLimitIntervalSec() const {
        return switchLimitIntervalSec;
    }
};
#endif

#define DEFAULT_INTERRUPT_PIN 2
#define RELAYS_COUNT_LOCATION 0
#define CONTROLLER_ID_LOCATION (RELAYS_COUNT_LOCATION + sizeof (uint8_t))
#define CONTROL_INTERRUPT_PIN_LOCATION (CONTROLLER_ID_LOCATION + sizeof (uint32_t))
#define STATE_FIX_SETTINGS_LOCATION (CONTROL_INTERRUPT_PIN_LOCATION + sizeof (uint8_t))
#ifdef MEM_32KB
    #define STATE_SWITCH_COUNT_SETTINGS_LOCATION (STATE_FIX_SETTINGS_LOCATION + sizeof (StateFixSettings))
    #define RELAYS_SETTINGS_START_LOCATION (STATE_SWITCH_COUNT_SETTINGS_LOCATION + sizeof (SwitchCountingSettings))
#else
    #define RELAYS_SETTINGS_START_LOCATION (STATE_FIX_SETTINGS_LOCATION + sizeof (StateFixSettings))
#endif
#define DEFAULT_RELAYS_COUNT 0
#define MAX_RELAYS_COUNT 16
#define RELAY_PIN_BITS_START 0
#define RELAY_PIN_BITS_LENGTH 5
#define RELAY_PIN_BITS_MASK BF_MASK(RELAY_PIN_BITS_START, RELAY_PIN_BITS_LENGTH)
#define RELAY_DISABLED_PIN BF_MASK(0, RELAY_PIN_BITS_LENGTH)
#define RELAY_INVERSED_BIT (RELAY_PIN_BITS_START + RELAY_PIN_BITS_LENGTH)
#define RELAY_INVERSED_BIT_MASK (1 << RELAY_INVERSED_BIT)
#define RELAY_SWITCH_BY_PUSH_BIT (RELAY_INVERSED_BIT + 1)
#define RELAY_SWITCH_BY_PUSH_BIT_MASK (1 << RELAY_SWITCH_BY_PUSH_BIT)

const uint8_t FORBIDEN_PINS[] = {0, 1, 11, 12, 13};

inline bool isPinAllowed(uint8_t pin) {
    for (unsigned char i : FORBIDEN_PINS) {
        if (pin == i) {
            return false;
        }
    }
    return true;
}

struct PinSettings {
private:
    uint8_t pinSettings;
public:
    explicit PinSettings(uint8_t settingsRaw) : pinSettings(settingsRaw) {}
    PinSettings() : pinSettings(RELAY_DISABLED_PIN) {}
    [[nodiscard]] inline uint8_t getPin() const {
        return (pinSettings & RELAY_PIN_BITS_MASK);
    }
    [[nodiscard]] inline bool isEnabled() const {
        return (pinSettings & RELAY_PIN_BITS_MASK) != RELAY_PIN_BITS_MASK;
    }
    [[nodiscard]] inline bool isBitSet(uint8_t mask) const {
        return (pinSettings & mask) != 0;
    }
    [[nodiscard]] inline bool isInversed() const {
        return isBitSet(RELAY_INVERSED_BIT_MASK);
    }
    inline void pinSettingsSetBit(bool set, uint8_t mask) {
        pinSettings = (pinSettings & ~mask) | (set ? mask : 0);
    }
    [[nodiscard]] inline uint8_t getRaw() const {
        return pinSettings;
    }
    [[nodiscard]] inline bool isAllowedPin() const {
        return isPinAllowed(getPin());
    }
};

struct RelaySettings {
private:
    PinSettings setPinSettings;
    PinSettings monitorPinSettings;
    PinSettings controlPinSettings;
public:
    RelaySettings(uint8_t setPinSettingsRaw, uint8_t monitorPinSettingsRaw, uint8_t controlPinSettingsRaw) :
        setPinSettings(setPinSettingsRaw), monitorPinSettings(monitorPinSettingsRaw), controlPinSettings(controlPinSettingsRaw) {}
    RelaySettings() : setPinSettings(), monitorPinSettings(), controlPinSettings() {}
    [[nodiscard]] inline const PinSettings& getSetPinSettings() const {
        return setPinSettings;
    }
    [[nodiscard]] inline const PinSettings& getMonitorPinSettings() const {
        return monitorPinSettings;
    }
    [[nodiscard]] inline const PinSettings& getControlPinSettings() const {
        return controlPinSettings;
    }
    [[nodiscard]] inline bool isControlPinSwitchByPush() const {
        return controlPinSettings.isBitSet(RELAY_SWITCH_BY_PUSH_BIT_MASK);
    }
};


struct SettingsPtr;


class Settings {
public:
    void load();
    [[nodiscard]] inline uint8_t getRelaysCount() const { return relaysCount; };
    [[nodiscard]] inline const RelaySettings& getRelaySettingsRef(uint8_t relayIdx) const { return relaySettings[relayIdx]; };
    [[nodiscard]] inline uint32_t getControllerId() const { return controllerId; }
    [[nodiscard]] inline uint8_t getControlInterruptPin() const { return controlInterruptPin; }
    [[nodiscard]] SettingsPtr getRelaysSettingsPtr() const;
    [[nodiscard]] inline const StateFixSettings &getStateFixSettings() const { return stateFixSettings; }
#ifdef MEM_32KB
    [[nodiscard]] inline const SwitchCountingSettings& getSwitchCountingSettingsRef() const { return switchCountingSettings; }
#endif
    [[nodiscard]] bool isReady() const  { return ready; }
    bool saveControlInterruptPin(uint8_t value);
    uint8_t saveRelaySettings(RelaySettings settings[], uint8_t count);
    void saveControllerId(uint32_t value);
    void saveStateFixSettings(const StateFixSettings &stateFixSettings);
#ifdef MEM_32KB
    void saveSwitchCountingSettings(const SwitchCountingSettings &stateFixSettings);
#endif
    inline void setOnSettingsChanged(void (*value)()) { onSettingsChanged = value; }

private:
    bool ready = false;
    uint8_t relaysCount;
    uint32_t controllerId;
    uint8_t controlInterruptPin = DEFAULT_INTERRUPT_PIN;
    StateFixSettings stateFixSettings;
    RelaySettings relaySettings[MAX_RELAYS_COUNT];
#ifdef MEM_32KB
    SwitchCountingSettings switchCountingSettings;
#endif
    void (*onSettingsChanged)() = nullptr;
};

struct SettingsPtr {
private:
    const Settings *settings;
public:
    explicit SettingsPtr(const Settings *settings) : settings(settings) {}
    SettingsPtr() : settings(nullptr) {}
    SettingsPtr(SettingsPtr &src) : settings(src.settings) {}
    [[nodiscard]] inline uint8_t getRelaysCount() const { return settings->getRelaysCount(); }
    [[nodiscard]] inline const RelaySettings& getRelaySettingsRef(uint8_t relayIdx) const { return settings->getRelaySettingsRef(relayIdx); }
    [[nodiscard]] inline uint8_t getControlInterruptPin() const { return settings->getControlInterruptPin(); }
    [[nodiscard]] inline const StateFixSettings& getStateFixSettings() const { return settings->getStateFixSettings(); }
#ifdef MEM_32KB
    [[nodiscard]] inline const SwitchCountingSettings& getSwitchCountingSettingsRef() const { return settings->getSwitchCountingSettingsRef(); }
#endif
};

#endif //RELAYCONTROLLER_SETTINGS_H
