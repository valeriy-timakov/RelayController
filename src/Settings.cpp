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
    for (uint8_t i = 0; i < relaysCount; i++) {
        EEPROM.get(RELAYS_SETTINGS_START_LOCATION + i * sizeof (RelaySettings), relaySettings[i]);
    }
    ready = true;
}

void Settings::saveRelaySettings(uint8_t relayIdx, uint8_t setPinSettingsRaw, uint8_t monitorPinSettingsRaw, uint8_t controlPinSettingsRaw) {
    relaySettings[relayIdx] = RelaySettings(setPinSettingsRaw, monitorPinSettingsRaw, controlPinSettingsRaw);
    EEPROM.put(RELAYS_SETTINGS_START_LOCATION + relayIdx * sizeof (RelaySettings), relaySettings[relayIdx]);
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
    return count;
}

void Settings::saveControllerId(uint32_t controllerId) {
    Settings::controllerId = controllerId;
    EEPROM.put(CONTROLLER_ID_LOCATION, controllerId);
}

const uint8_t ALLOWED_INTERRUPT_PINS[] = {2, 3};

bool Settings::saveControlInterruptPin(uint8_t controlInterruptPin) {
    bool allowed = false;
    for (uint8_t i = 0; i < sizeof (ALLOWED_INTERRUPT_PINS); i++) {
        if (ALLOWED_INTERRUPT_PINS[i] == controlInterruptPin) {
            allowed = true;
            break;
        }
    }
    if (!allowed) return false;
    Settings::controlInterruptPin = controlInterruptPin;
    EEPROM.put(CONTROL_INTERRUPT_PIN_LOCATION, controlInterruptPin);
    return true;
}

uint8_t Settings::getRelaysCount() const {
    return relaysCount;
}

RelaySettings Settings::getRelaySettingsCopy(uint8_t relayIdx) const {
    return relaySettings[relayIdx];
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



