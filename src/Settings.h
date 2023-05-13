//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_SETTINGS_H
#define RELAYCONTROLLER_SETTINGS_H

#include "Arduino.h"
#include "utils.h"


#define DEFAULT_INTERRUPT_PIN 2
#define RELAYS_COUNT_LOCATION 0
#define CONTROLLER_ID_LOCATION (RELAYS_COUNT_LOCATION + sizeof (uint8_t))
#define CONTROL_INTERRUPT_PIN_LOCATION (CONTROLLER_ID_LOCATION + sizeof (uint32_t))
#define RELAYS_SETTINGS_START_LOCATION (CONTROL_INTERRUPT_PIN_LOCATION + sizeof (uint8_t))
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

inline uint8_t pinSettingsGetPin(uint8_t pinSettings) {
    return (pinSettings & RELAY_PIN_BITS_MASK) >> RELAY_PIN_BITS_START;
}
inline bool pinSettingsIsEnabled(uint8_t pinSettings) {
    return (pinSettings & RELAY_PIN_BITS_MASK) != RELAY_PIN_BITS_MASK;
}
inline bool pinSettingsIsInversed(uint8_t pinSettings) {
    return (pinSettings & RELAY_INVERSED_BIT_MASK) != 0;
}
inline void pinSettingsSetPin(uint8_t &pinSettings, uint8_t pin) {
    pinSettings = (pinSettings & ~RELAY_PIN_BITS_MASK) | ((pin << RELAY_PIN_BITS_START) & RELAY_PIN_BITS_MASK);
}
inline void pinSettingsSetDisabled(uint8_t &pinSettings) {
    pinSettingsSetPin(pinSettings, RELAY_DISABLED_PIN);
}
inline void pinSettingsSetBit(uint8_t &pinSettings, bool set, uint8_t mask) {
    pinSettings = (pinSettings & ~mask) | (set ? mask : 0);
}

struct PinSettings {
private:
    uint8_t pinSettings;
public:
    PinSettings(uint8_t settingsRaw) : pinSettings(settingsRaw) {}
    PinSettings() : pinSettings(RELAY_DISABLED_PIN) {}
    inline uint8_t getPin() const {
        return (pinSettings & RELAY_PIN_BITS_MASK) >> RELAY_PIN_BITS_START;
    }
    inline bool isEnabled() const {
        return (pinSettings & RELAY_PIN_BITS_MASK) != RELAY_PIN_BITS_MASK;
    }
    inline bool isBitSet(uint8_t mask) const {
        return (pinSettings & mask) != 0;
    }
    inline bool isInversed() const {
        return isBitSet(RELAY_INVERSED_BIT_MASK);
    }
    inline void pinSettingsSetPin(uint8_t pin) {
        pinSettings = (pinSettings & ~RELAY_PIN_BITS_MASK) | ((pin << RELAY_PIN_BITS_START) & RELAY_PIN_BITS_MASK);
    }
    inline void pinSettingsSetDisabled() {
        pinSettingsSetPin(RELAY_DISABLED_PIN);
    }
    inline void pinSettingsSetBit(bool set, uint8_t mask) {
        pinSettings = (pinSettings & ~mask) | (set ? mask : 0);
    }
    inline void pinSettingsSetInversed(bool inversed) {
        pinSettingsSetBit(inversed, RELAY_INVERSED_BIT_MASK);
    }
    inline uint8_t getRaw() const {
        return pinSettings;
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
    inline const PinSettings& getSetPinSettings() const {
        return setPinSettings;
    }
    inline PinSettings& getSetPinSettingsMut() {
        return setPinSettings;
    }
    inline const PinSettings& getMonitorPinSettings() const {
        return monitorPinSettings;
    }
    inline PinSettings& getMonitorPinSettingsMut() {
        return monitorPinSettings;
    }
    inline const PinSettings& getControlPinSettings() const {
        return controlPinSettings;
    }
    inline PinSettings& getControlPinSettingsMut() {
        return controlPinSettings;
    }
    inline bool isControlPinSwitchByPush() const {
        return controlPinSettings.isBitSet(RELAY_SWITCH_BY_PUSH_BIT_MASK);
    }
    inline void setControlPinSwitchByPush(bool switchByPush) {
        controlPinSettings.pinSettingsSetBit(switchByPush, RELAY_SWITCH_BY_PUSH_BIT_MASK);
    }
};


struct SettingsPtr;


class Settings {
public:
    void load();
    uint8_t getRelaysCount() const;
    RelaySettings getRelaySettingsCopy(uint8_t relayIdx) const;
    const RelaySettings& getRelaySettingsRef(uint8_t relayIdx) const;
    uint32_t getControllerId() const;
    bool isReady() const;
    bool saveRelaysCount(uint8_t value);
    void saveControlInterruptPin(uint8_t controlInterruptPin);
    void saveRelaySettings(uint8_t relayIdx, uint8_t setPinSettingsRaw, uint8_t monitorPinSettingsRaw, uint8_t controlPinSettingsRaw);
    void saveControllerId(uint32_t controllerId);
    uint8_t getControlInterruptPin() const;
    SettingsPtr getRelaysSettingsPtr() const;


private:
    bool ready = false;
    uint8_t relaysCount;
    uint32_t controllerId;
    uint8_t controlInterruptPin = DEFAULT_INTERRUPT_PIN;
    RelaySettings relaySettings[MAX_RELAYS_COUNT];
};

struct SettingsPtr {
private:
    const Settings *settings;
public:
    SettingsPtr(const Settings *settings) : settings(settings) {}
    SettingsPtr() : settings(nullptr) {}
    SettingsPtr(SettingsPtr &src) : settings(src.settings) {}
    inline uint8_t getRelaysCount() const { return settings->getRelaysCount(); }
    inline const RelaySettings& getRelaySettingsRef(uint8_t relayIdx) const { return settings->getRelaySettingsRef(relayIdx); }
    inline uint8_t getControlInterruptPin() const { return settings->getControlInterruptPin(); }
};

#endif //RELAYCONTROLLER_SETTINGS_H
