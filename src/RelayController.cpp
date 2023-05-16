//
// Created by valti on 06.05.2023.
//

#include "RelayController.h"

bool lastInterruptPinHigh = false;

SettingsPtr _settings;
uint16_t _lastControlState = 0;
uint16_t _lastRelayState = 0;
uint16_t _temporaryDisabledControls = 0;


void switchRelayState(const RelaySettings &settings, uint8_t i);

bool getLastControlState(uint8_t relayIdx) {
    return CHECK_BIT(_lastControlState, relayIdx);
}

void setLastControlState(uint8_t relayIdx, bool swithedOn) {
    setBit(_lastControlState, relayIdx, swithedOn);
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
            digitalWrite(setPinSettings.getPin(), setPinSettings.isInversed() == !swithedOn ? HIGH : LOW);
        }
        setLastRelayState(relayIdx, swithedOn);
    }
}

void checkAndProcessChanges() {
    for (uint8_t i = 0; i < _settings.getRelaysCount(); i++) {
        const RelaySettings &relaySettings = _settings.getRelaySettingsRef(i);
        const PinSettings &ctrlPinSettings = relaySettings.getControlPinSettings();
        const PinSettings &setPinSettings = relaySettings.getSetPinSettings();
        if (ctrlPinSettings.isEnabled() && setPinSettings.isEnabled() && !_isControlTemporaryDisabled(i)) {
            bool ctrlPinSet = checkPinState(ctrlPinSettings);
            bool lastSwithedOn = getLastControlState(i);
            if (lastSwithedOn != ctrlPinSet) {
                if (relaySettings.isControlPinSwitchByPush()) {
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
}

void RelayController::idle() {
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

