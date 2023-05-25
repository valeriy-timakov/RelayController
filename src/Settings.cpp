//
// Created by valti on 06.05.2023.
//

#include "Settings.h"
#include <EEPROM.h>


void Settings::load() {
    relaysCount = EEPROM.read(RELAYS_COUNT_LOCATION);
    if (relaysCount > MAX_RELAYS_COUNT) {
        relaysCount = DEFAULT_RELAYS_COUNT;
        EEPROM.update(RELAYS_COUNT_LOCATION, relaysCount);
    }
    EEPROM.get(CONTROLLER_ID_LOCATION, controllerId);
    EEPROM.get(CONTROL_INTERRUPT_PIN_LOCATION, controlInterruptPin);
    EEPROM.get(STATE_FIX_SETTINGS_LOCATION, stateFixSettings);
    for (uint8_t i = 0; i < relaysCount; i++) {
        EEPROM.get(RELAYS_SETTINGS_START_LOCATION + i * sizeof (RelaySettings), relaySettings[i]);
    }
    ready = true;
}

uint8_t Settings::saveRelaySettings(RelaySettings settings[], uint8_t count) {
    if (count > MAX_RELAYS_COUNT) {
        count = MAX_RELAYS_COUNT;
    }
    relaysCount = count;
    EEPROM.update(RELAYS_COUNT_LOCATION, relaysCount);
    for (uint8_t i = 0; i < count; i++) {
        relaySettings[i] = settings[i];
        EEPROM.put(RELAYS_SETTINGS_START_LOCATION + i * sizeof (RelaySettings), relaySettings[i]);
    }
    if (onSettingsChanged != nullptr) {
        onSettingsChanged();
    }
    return count;
}

void Settings::saveControllerId(uint32_t value) {
    controllerId = value;
    EEPROM.put(CONTROLLER_ID_LOCATION, value);
}

const uint8_t ALLOWED_INTERRUPT_PINS[] = {2, 3};

bool Settings::saveControlInterruptPin(uint8_t value) {
    bool allowed = false;
    for (unsigned char i : ALLOWED_INTERRUPT_PINS) {
        if (i == value) {
            allowed = true;
            break;
        }
    }
    if (!allowed) return false;
    controlInterruptPin = value;
    EEPROM.put(CONTROL_INTERRUPT_PIN_LOCATION, value);
    return true;
}

void Settings::saveStateFixSettings(const StateFixSettings &value) {
    stateFixSettings = value;
    EEPROM.put(STATE_FIX_SETTINGS_LOCATION, stateFixSettings);
}

uint8_t Settings::getRelaysCount() const {
    return relaysCount;
}

const RelaySettings &Settings::getRelaySettingsRef(uint8_t relayIdx) const {
    return relaySettings[relayIdx];
}

uint32_t Settings::getControllerId() const {
    return controllerId;
}

bool Settings::isReady() const {
    return ready;
}

uint8_t Settings::getControlInterruptPin() const {
    return controlInterruptPin;
}

SettingsPtr Settings::getRelaysSettingsPtr() const {
    return SettingsPtr(this);
}

const StateFixSettings &Settings::getStateFixSettings() const {
    return stateFixSettings;
}

void Settings::setOnSettingsChanged(void (*value)())  {
    onSettingsChanged = value;
}



