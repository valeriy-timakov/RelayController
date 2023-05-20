//
// Created by valti on 06.05.2023.
//

#include "Communicator.h"
#include "utils.h"

#define MAIN_CODE_POSITION 0
#define INSTRUCTION_CODE_POSITION (MAIN_CODE_POSITION + 1)
#define INSTRUCTION_DATA_START_CODE_POSITION (INSTRUCTION_CODE_POSITION + 1)
#define SETTINGS_SIZE_PER_RELAY 3
#define SET_RELAY_STATE_DATA_SIZE 2
#define SET_RELAY_STATE_DATA_COUNT_IN_BYTE (8 / SET_RELAY_STATE_DATA_SIZE)



size_t sendSerial(bool value, Stream &serial = Serial) {
    return serial.write(value ? 1 : 0);
}

size_t sendSerial(uint8_t value, Stream &serial = Serial) {
    return serial.write(value);
}

size_t sendSerial(InstructionCode value, Stream &serial = Serial) {
    return sendSerial((uint8_t)value, serial);
}

size_t sendSerial(InstructionDataCode value, Stream &serial = Serial) {
    return sendSerial((uint8_t)value, serial);
}

size_t sendSerial(ErrorCode buffer, Stream &serial = Serial) {
    return serial.write(buffer);
}

size_t sendSerial(uint16_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    sum += serial.write(value >> 8);
    sum += serial.write(value & 0xFF);
    return sum;
}

size_t sendSerial(uint32_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    uint8_t shift = 24;
    for (uint8_t i = 0; i < 3; i++) {
        sum += serial.write((uint8_t) ((value >> shift) & 0xFF) );
        shift -= 8;
    }
    sum += serial.write(value & 0xFF);
    return sum;
}

size_t sendSerial(uint64_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    uint8_t shift = 56;
    for (uint8_t i = 0; i < 7; i++) {
        sum += serial.write((uint8_t) ((value >> shift) & 0xFF) );
        shift -= 8;
    }
    sum += serial.write((uint8_t)(value & 0xFF));
    return sum;
}

size_t sendSerial(const uint8_t *buffer, size_t size, Stream &serial = Serial) {
    return serial.write(buffer, size);
}

size_t sendSerial(const char *buffer, Stream &serial = Serial) {
    return serial.write(buffer);
}

void Communicator::setup() {
    sendSerial((uint64_t)0L);
    if (!settings.isReady()) {
        settings.load();
    }
}

void Communicator::idle() {
    while (readBinaryCommand()) {
        processBinaryInstruction();
    }
}

void clearSerial() {
    while (Serial.available()) {
        Serial.read();
    }
}

void sendStartAnswer(InstructionDataCode code) {
    sendSerial(IC_NONE);
    sendSerial(code);
}

void sendError(ErrorCode code) {
    sendSerial(IC_NONE);
    sendSerial(IC_ERROR);
    sendSerial(code);
}

void sendSuccess() {
    sendSerial(IC_NONE);
    sendSerial(IC_SUCCESS);
}

void sendSuccess(uint8_t value) {
    sendSerial(IC_NONE);
    sendSerial(IC_SUCCESS);
    sendSerial(value);
}

bool Communicator::readBinaryCommand() {
    if (commandParsed && !commandPocessed) {
        return true;
    }
    uint8_t available = Serial.available();
    if (!available) {
        return false;
    }
    auto currTime = (uint32_t) millis();
    bool commandReady = lastPacketSize > 0 && lastPacketTime > 0 &&  (available - lastPacketSize) == 0 && (currTime - lastPacketTime) > MAX_COMMAND_READ_TIME;
    if (available != lastPacketSize) {
        lastPacketTime = currTime;
    }
    lastPacketSize = available;
    if (!commandReady) {
        return false;
    }
    lastPacketSize = 0;
    lastPacketTime = 0;
    int r = Serial.read();
    if (r != IC_NONE) {
        clearSerial();
        sendError(E_INSTRUCTION_WRONG_START);
        return false;
    }
    commandParsed = false;
    available = Serial.available();
    if (available == 0) {
        sendError(E_COMMAND_EMPTY);
        return false;
    }
    if (available > CMD_BUFF_SIZE) {
        clearSerial();
        sendError(E_COMMAND_SIZE_OVERFLOW);
        return false;
    }
    Serial.readBytes(cmdBuff, available);
    cmdBuffSize = available;
    bool commandUnrecognized;
    if (cmdBuffSize < 2) {
        commandUnrecognized = true;
    } else {
        uint8_t firstCode = cmdBuff[MAIN_CODE_POSITION];
        commandUnrecognized = firstCode != IC_READ && firstCode != IC_SET;
    }
    if (commandUnrecognized) {
        sendSerial(cmdBuff, cmdBuffSize);
        sendError(E_INSTRUCTION_UNRECOGIZED);
        return false;
    }
    commandParsed = true;
    commandPocessed = false;
    return true;
}

void Communicator::processBinaryInstruction() {
    uint8_t mainCode = cmdBuff[MAIN_CODE_POSITION];
    cmdBuffCurrPos = INSTRUCTION_DATA_START_CODE_POSITION;
    ErrorCode result;
    switch (mainCode) {
        case IC_READ:
            result = processBinaryRead();
            break;
        case IC_SET:
            if (cmdBuffSize < 3) {
                result = E_REQUEST_DATA_NO_VALUE;
                break;
            }
            result = processBinarySet();
            if (result == OK) {
                sendSuccess();
            }
            break;
    }
    if (result != OK) {
        if (result < E_UNDEFINED_CODE) {
            sendError(result);
        } else {
            sendSuccess(result);
        }
    }
    commandPocessed = true;
}

ErrorCode Communicator::processBinaryRead() {
    auto code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {
        case IDC_SETTINGS:
            sendSettings();
            return OK;
        case IDC_STATE:
            return sendState();
        case IDC_ID:
            return sendId();
        case IDC_ALL:
            return sendAll();
        case IDC_RELAY_STATE:
            return sendRelayState();
        case IDC_RELAY_DISABLED_TEMP:
            return sendRelayDisabledTemp();
        case IDC_RELAY_SWITCHED_ON:
            return sendRelaySwitchedOn();
        case IDC_RELAY_MONITOR_ON:
            return sendRelayMonitorOn();
        case IDC_RELAY_CONTROL_ON:
            return sendRelayControlOn();
        case IDC_INTERRUPT_PIN:
            return sendInterruptPin(true);
        case IDC_CONTACT_READY_WAIT_DELAY:
            return sendContactReadyWaitDelay();
        case IDC_SWITCH_COUNT_INTERVAL_SEC:
            return sendSwitchCountIntervalSec();
#ifdef MEM_32KB
            case IDC_MAX_SWITCH_COUNT:
            return sendMaxSwitchCount();
#endif
    }
    return E_UNDEFINED_OPERATION;
}

ErrorCode Communicator::processBinarySet() {
    auto code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {
        case IDC_SETTINGS:
            return (ErrorCode) saveSettings();
        case IDC_STATE:
            return saveState();
        case IDC_ID:
            return saveId();
        case IDC_ALL:
            return saveAll();
        case IDC_RELAY_STATE:
            return saveRelayState();
        case IDC_RELAY_DISABLED_TEMP:
            return saveRelayDisabledTemp();
        case IDC_RELAY_SWITCHED_ON:
            return saveRelaySwitchedOn();
        case IDC_INTERRUPT_PIN:
            return saveInterruptPin();
        case IDC_CONTACT_READY_WAIT_DELAY:
            return saveContactReadyWaitDelay();
        case IDC_SWITCH_COUNT_INTERVAL_SEC:
            return saveSwitchCountIntervalSec();
#ifdef MEM_32KB
            case IDC_MAX_SWITCH_COUNT:
            return saveMaxSwitchCount();
        case IDC_CLEAR_SWITCH_COUNT:
            return clearSwitchCount();
#endif
    }
    return E_UNDEFINED_OPERATION;
}

void Communicator::sendSettings(bool addResultCode) {
    uint8_t relayCount = settings.getRelaysCount();
    if (addResultCode) {
        sendStartAnswer(IDC_SETTINGS);
        sendSerial(relayCount);
    }
    for (uint8_t i = 0; i < relayCount; i++) {
        const RelaySettings &relay = settings.getRelaySettingsRef(i);
        sendSerial(relay.getSetPinSettings().getRaw());
        sendSerial(relay.getMonitorPinSettings().getRaw());
        sendSerial(relay.getControlPinSettings().getRaw());
    }
}

uint8_t Communicator::saveSettings() {
    uint8_t relayCount = 0;
    ErrorCode res = readRelayCountFromCmdBuff(relayCount);
    if (res != OK) return res;
    if (relayCount * SETTINGS_SIZE_PER_RELAY + cmdBuffCurrPos > cmdBuffSize) return E_RELAY_COUNT_AND_DATA_MISMATCH;
    RelaySettings relaySettings[relayCount];
    for (uint8_t i = 0; i < relayCount; i++) {
        relaySettings[i] = RelaySettings(
            cmdBuff[cmdBuffCurrPos],
            cmdBuff[cmdBuffCurrPos + 1],
            cmdBuff[cmdBuffCurrPos + 2]
        );
        cmdBuffCurrPos += 3;
        if (!relaySettings[i].getSetPinSettings().isAllowedPin()){
            return E_RELAY_NOT_ALLOWED_PIN_USED | relaySettings[i].getSetPinSettings().getPin();}
        if (!relaySettings[i].getMonitorPinSettings().isAllowedPin()){
            return E_RELAY_NOT_ALLOWED_PIN_USED | relaySettings[i].getMonitorPinSettings().getPin();}
        if (!relaySettings[i].getControlPinSettings().isAllowedPin()){
            return E_RELAY_NOT_ALLOWED_PIN_USED | relaySettings[i].getControlPinSettings().getPin();}
    }
    uint8_t savedCount = settings.saveRelaySettings(relaySettings, relayCount);
    return savedCount | E_UNDEFINED_CODE;
}

ErrorCode Communicator::sendState(bool addResultCode) {
    uint8_t count = settings.getRelaysCount();
    uint8_t pairsCount = count / 2 + count % 2;
    uint8_t result[pairsCount];
    for (uint8_t i = 0; i < count; i += 2) {
        uint8_t tmpResult1 = readRelayStateBits(i);
        uint8_t tmpResult2 = i + 1 < count ? readRelayStateBits(i + 1) : 0;
        result[i / 2] = readRelayStateBits(tmpResult2 << 4 | tmpResult1);
    }
    if (addResultCode) {
        sendStartAnswer(IDC_STATE);
        sendSerial(count);
    }
    for (uint8_t i = 0; i < pairsCount; i++) {
        sendSerial(result[i]);
    }
    return OK;
}

ErrorCode Communicator::saveState() {
    uint8_t providedCount = 0;
    ErrorCode res = readRelayCountFromCmdBuff(providedCount);
    if (res != OK) return res;
    if (providedCount != settings.getRelaysCount()) return E_RELAY_COUNT_AND_DATA_MISMATCH;
    if (cmdBuffSize < cmdBuffCurrPos + SET_RELAY_STATE_DATA_SIZE * providedCount + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t* data = cmdBuff + cmdBuffCurrPos;
    for (uint8_t i = 0; i < providedCount; i++) {
        uint8_t command = data[i / SET_RELAY_STATE_DATA_COUNT_IN_BYTE];
        uint8_t shift = (i % SET_RELAY_STATE_DATA_COUNT_IN_BYTE) * SET_RELAY_STATE_DATA_SIZE;
        relayController.setRelayState(i, CHECK_BIT(command, shift));
        relayController.setControlTemporaryDisabled(i, CHECK_BIT(command, shift + 1));
    }
    return OK;
}

ErrorCode Communicator::sendId(bool addResultCode) {
    if (addResultCode) {
        sendStartAnswer(IDC_ID);
    }
    sendSerial(settings.getControllerId());
    return OK;
}

ErrorCode Communicator::saveId() {
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint32_t)) return E_REQUEST_DATA_NO_VALUE;
    uint32_t controllerId = 0;
    ErrorCode res = readUint32FromCommandBuffer(controllerId);
    if (res != OK) return res;
    settings.saveControllerId(controllerId);
    return OK;
}

ErrorCode Communicator::sendInterruptPin(bool addResultCode) {
    if (addResultCode) {
        sendStartAnswer(IDC_INTERRUPT_PIN);
    }
    sendSerial(settings.getControlInterruptPin());
    return OK;
}

ErrorCode Communicator::saveInterruptPin() {
    uint8_t pin = 0;
    ErrorCode res = readUint8FromCmdBuff(pin);
    if (res != OK) return res;
    return settings.saveControlInterruptPin(pin) ? OK : E_CONTROL_INTERRUPTED_PIN_NOT_ALLOWED_VALUE;
}

ErrorCode Communicator::sendContactReadyWaitDelay() {
    sendStartAnswer(IDC_CONTACT_READY_WAIT_DELAY);
    sendSerial(relayController.getContactReadyWaitDelay());
    return OK;
}

ErrorCode Communicator::saveContactReadyWaitDelay() {
    uint16_t value = 0;
    ErrorCode res = readUint16FromCmdBuff(value);
    if (res != OK) return res;
    relayController.setContactReadyWaitDelay(value);
    return OK;
}

ErrorCode Communicator::sendSwitchCountIntervalSec() {
    sendStartAnswer(IDC_SWITCH_COUNT_INTERVAL_SEC);
    sendSerial(relayController.getSwitchLimitIntervalSec());
    return OK;
}

ErrorCode Communicator::saveSwitchCountIntervalSec() {
    uint16_t value = 0;
    ErrorCode res = readUint16FromCmdBuff(value);
    if (res != OK) return res;
    relayController.setSwitchLimitIntervalSec(value);
    return OK;
}
#ifdef MEM_32KB
ErrorCode Communicator::sendMaxSwitchCount() {
    uint8_t relayIdx = 0;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIdx);
    if (res != OK) return res;
    sendStartAnswer(IDC_MAX_SWITCH_COUNT);
    sendSerial(relayController.getMaxSwitchCount(relayIdx));
    return OK;
}

ErrorCode Communicator::saveMaxSwitchCount() {
    uint8_t relayIdx = 0;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIdx);
    if (res != OK) return res;
    uint8_t value = 0;
    res = readUint8FromCmdBuff(value);
    if (res != OK) return res;
    relayController.setMaxSwitchCount(relayIdx, value);
    return OK;
}

ErrorCode Communicator::clearSwitchCount() {
    uint8_t relayIdx = 0;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIdx);
    if (res != OK) return res;
    relayController.clearSwitchCount(relayIdx);
    return OK;
}

#endif

ErrorCode Communicator::sendAll() {
    sendStartAnswer(IDC_ALL);
    sendId(false);
    sendInterruptPin(false);
    sendSerial(settings.getRelaysCount());
    sendSettings(false);
    sendState(false);
    return OK;
}

ErrorCode Communicator::saveAll() {
    uint8_t relayCount = 0;
    ErrorCode res = readRelayCountFromCmdBuff(relayCount);
    if (res != OK) return res;
    if (cmdBuffCurrPos + sizeof(uint32_t) + relayCount * SETTINGS_SIZE_PER_RELAY + SET_RELAY_STATE_DATA_SIZE * relayCount + 1 > cmdBuffSize) {
        return E_REQUEST_DATA_NO_VALUE;
    }

    saveId();
    saveInterruptPin();
    saveSettings();
    saveState();

    return OK;
}

ErrorCode Communicator::send(uint8_t(*getter)(Communicator*, uint8_t), InstructionDataCode dataCode) {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIndex);
    if (res != OK) return res;
    uint8_t value = getter(this, relayIndex);
    sendStartAnswer(dataCode);
    if (value <= 0x0f) {
        sendSerial((uint8_t)(((value & 0x0F) << 4) | relayIndex));
    } else {
        sendSerial(value);
        sendSerial(relayIndex);
    }
    return OK;
}

ErrorCode Communicator::save(void(*setter)(Communicator*, uint8_t, uint8_t)) {
    uint8_t cmdData = 0;
    ErrorCode res = readUint8FromCmdBuff(cmdData);
    if (res != OK) return res;
    uint8_t relayIndex = cmdData & 0x0f;
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = cmdData >> 4;
    setter(this, relayIndex, value);
    return OK;
}

ErrorCode Communicator::sendRelayState() {
    return send([]  (Communicator *communicator, uint8_t relayIndex) {
        uint8_t value = communicator-> readRelayStateBits(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_STATE);
}

ErrorCode Communicator::saveRelayState() {
    return save([] (Communicator *communicator, uint8_t relayIndex, uint8_t cmdData) {
        communicator->relayController.setRelayState(relayIndex, CHECK_BIT(cmdData, 0));
        communicator->relayController.setControlTemporaryDisabled(relayIndex, CHECK_BIT(cmdData, 1));
    });
}

ErrorCode Communicator::sendRelayDisabledTemp() {
    return send([]  (Communicator *communicator, uint8_t relayIndex) {
        uint8_t value = communicator-> relayController.isControlTemporaryDisabled(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_DISABLED_TEMP);
}

ErrorCode Communicator::saveRelayDisabledTemp() {
    return save([] (Communicator *communicator, uint8_t relayIndex, uint8_t cmdData) {
        communicator->relayController.setControlTemporaryDisabled(relayIndex, CHECK_BIT(cmdData, 0));
    });
}

ErrorCode Communicator::sendRelaySwitchedOn() {
    return send([]  (Communicator *communicator, uint8_t relayIndex) {
        uint8_t value = communicator-> relayController.getRelayLastState(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_SWITCHED_ON);
}

ErrorCode Communicator::saveRelaySwitchedOn() {
    return save([] (Communicator *communicator, uint8_t relayIndex, uint8_t cmdData) {
        communicator->relayController.setRelayState(relayIndex, CHECK_BIT(cmdData, 0));
    });
}

ErrorCode Communicator::sendRelayMonitorOn() {
    return send([]  (Communicator *communicator, uint8_t relayIndex) {
        uint8_t value = communicator-> relayController.checkRelayMonitoringState(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_MONITOR_ON);
}

ErrorCode Communicator::sendRelayControlOn() {
    return send([]  (Communicator *communicator, uint8_t relayIndex) {
        uint8_t value = communicator-> relayController.checkControlPinState(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_CONTROL_ON);
}


ErrorCode Communicator::readUint8FromCmdBuff(uint8_t &result) {
    if (cmdBuffSize < cmdBuffCurrPos + 1) return E_REQUEST_DATA_NO_VALUE;
    result = cmdBuff[cmdBuffCurrPos++];
    return OK;
}

ErrorCode Communicator::readUint16FromCmdBuff(uint16_t &result) {
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint32_t)) return E_REQUEST_DATA_NO_VALUE;
    result |= ((uint_fast16_t)cmdBuff[cmdBuffCurrPos++]) << 8;
    result |= cmdBuff[cmdBuffCurrPos++];
    return OK;
}

ErrorCode Communicator::readUint32FromCommandBuffer(uint32_t &result) {
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint32_t)) return E_REQUEST_DATA_NO_VALUE;
    result |= ((uint_fast32_t)cmdBuff[cmdBuffCurrPos++]) << 24;
    result |= ((uint_fast32_t)cmdBuff[cmdBuffCurrPos++]) << 16;
    result |= ((uint_fast32_t)cmdBuff[cmdBuffCurrPos++]) << 8;
    result |= cmdBuff[cmdBuffCurrPos++];
    return OK;
}

ErrorCode Communicator::readRelayIndexFromCmdBuff(uint8_t &result) {
    uint8_t res = 0;
    ErrorCode readRes = readUint8FromCmdBuff(res);
    if (readRes != OK) return readRes;
    if (res >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    result = res;
    return OK;
}

ErrorCode Communicator::readRelayCountFromCmdBuff(uint8_t &count) {
    uint8_t res = 0;
    ErrorCode readRes = readUint8FromCmdBuff(res);
    if (readRes != OK) return readRes;
    if (res >= MAX_RELAYS_COUNT) return E_RELAY_COUNT_OVERFLOW;
    count = res;
    return OK;
}

uint8_t Communicator::readRelayStateBits(uint8_t relayIndex) {
    uint8_t result = 0;
    setBit(result, 0, relayController.checkRelayMonitoringState(relayIndex));
    setBit(result, 1, relayController.getRelayLastState(relayIndex));
    setBit(result, 2, relayController.isControlTemporaryDisabled(relayIndex));
    setBit(result, 3, relayController.checkControlPinState(relayIndex));
    return result;
}


