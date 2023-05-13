//
// Created by valti on 06.05.2023.
//

#include "Settings.h"
#include <EEPROM.h>


void Settings::load() {
    relaysCount = EEPROM.read(RELAYS_COUNT_LOCATION);
    if (relaysCount == 0 || relaysCount > MAX_RELAYS_COUNT) {
        saveRelaysCount(DEFAULT_RELAYS_COUNT);
    }
    EEPROM.get(CONTROLLER_ID_LOCATION, controllerId);
    EEPROM.get(CONTROL_INTERRUPT_PIN_LOCATION, controlInterruptPin);
    for (uint8_t i = 0; i < relaysCount; i++) {
        EEPROM.get(RELAYS_SETTINGS_START_LOCATION + i * sizeof (RelaySettings), relaySettings[i]);
    }
    ready = true;
}

bool Settings::saveRelaysCount(uint8_t value) {
    if (relaysCount > MAX_RELAYS_COUNT) {
        return false;
    }
    relaysCount = value;
    EEPROM.update(RELAYS_COUNT_LOCATION, relaysCount);
}

void Settings::saveRelaySettings(uint8_t relayIdx, uint8_t setPinSettingsRaw, uint8_t monitorPinSettingsRaw, uint8_t controlPinSettingsRaw) {
    relaySettings[relayIdx] = RelaySettings(setPinSettingsRaw, monitorPinSettingsRaw, controlPinSettingsRaw);
    EEPROM.put(RELAYS_SETTINGS_START_LOCATION + relayIdx * sizeof (RelaySettings), relaySettings[relayIdx]);
}

void Settings::saveControllerId(uint32_t controllerId) {
    Settings::controllerId = controllerId;
    EEPROM.put(CONTROLLER_ID_LOCATION, controllerId);
}

void Settings::saveControlInterruptPin(uint8_t controlInterruptPin) {
    Settings::controlInterruptPin = controlInterruptPin;
    EEPROM.put(CONTROL_INTERRUPT_PIN_LOCATION, controlInterruptPin);
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



