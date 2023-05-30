//
// Created by valti on 06.05.2023.
//

#include "Server.h"
#include "utils.h"

#define MAIN_CODE_POSITION 0
#define INSTRUCTION_CODE_POSITION (MAIN_CODE_POSITION + 1)
#define INSTRUCTION_DATA_START_CODE_POSITION (INSTRUCTION_CODE_POSITION + 1)
#define SETTINGS_SIZE_PER_RELAY 3
#define SET_RELAY_STATE_DATA_SIZE 2
#define SET_RELAY_STATE_DATA_COUNT_IN_BYTE (8 / SET_RELAY_STATE_DATA_SIZE)


uint16_t Server::minCycleDuration = 0xffff;
uint16_t Server::maxCycleDuration = 0;
uint64_t Server::cyclesCount = 0;
uint64_t Server::lastCycleTime = 0;

void Server::setup() {
    sendSerial((uint64_t)0L);
    if (!settings.isReady()) {
        settings.load();
    }
}

void Server::idle() {
    updateStatistics();
    while (readBinaryCommand()) {
        processBinaryInstruction();
    }
}

void Server::updateStatistics() {
    uint64_t currTime = millis();
    uint16_t cycleDuration = currTime - lastCycleTime;
    lastCycleTime = currTime;
    if (cycleDuration > maxCycleDuration) {
        maxCycleDuration = cycleDuration;
    } else if (cycleDuration < minCycleDuration) {
        minCycleDuration = cycleDuration;
    }
    cyclesCount++;
}

void clearSerial() {
    while (Serial.available()) {
        Serial.read();
    }
}

void sendError(ErrorCode code) {
    sendSerial(IC_NONE);
    sendSerial(IC_ERROR);
    sendSerial(code);
}

void sendSuccess(InstructionDataCode code) {
    sendSerial(IC_NONE);
    sendSerial(IC_SUCCESS);
    sendSerial(code);
}

void sendSuccess(InstructionDataCode code, uint8_t value) {
    sendSerial(IC_NONE);
    sendSerial(IC_SUCCESS);
    sendSerial(code);
    sendSerial(value);
}

bool Server::readBinaryCommand() {
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

void Server::processBinaryInstruction() {
    uint8_t mainCode = cmdBuff[MAIN_CODE_POSITION];
    cmdBuffCurrPos = INSTRUCTION_DATA_START_CODE_POSITION;
    ErrorCode result;
    auto code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (mainCode) {
        case IC_READ:
            result = processBinaryRead(code);
            break;
        case IC_SET:
            if (cmdBuffSize < 3) {
                result = E_REQUEST_DATA_NO_VALUE;
                break;
            }
            result = processBinarySet(code);
            if (result == OK) {
                sendSuccess(code);
            }
            break;
        default:
            result = E_INSTRUCTION_UNRECOGIZED;
            break;
    }
    if (result != OK) {
        if (result < E_UNDEFINED_CODE) {
            sendError(result);
        } else {
            sendSuccess(code, result);
        }
    }
    commandPocessed = true;
}

ErrorCode Server::processBinaryRead(InstructionDataCode code) {
    switch (code) {
        case IDC_SETTINGS:
            sendSettings();
            return OK;
        case IDC_STATE:
            return sendState();
        case IDC_ID:
            return sendId();
#ifdef MEM_32KB
        case IDC_ALL:
            return sendAll();
#endif
        case IDC_RELAY_STATE:
            return sendRelayState();
        case IDC_STATE_FIX_SETTINGS:
            return sendStateFixSettings();
        case IDC_REMOTE_TIMESTAMP:
            return sendRemoteTimestamp();
#ifdef MEM_32KB
        case IDC_RELAY_DISABLED_TEMP:
            return sendRelayDisabledTemp();
        case IDC_RELAY_SWITCHED_ON:
            return sendRelaySwitchedOn();
        case IDC_RELAY_MONITOR_ON:
            return sendRelayMonitorOn();
        case IDC_RELAY_CONTROL_ON:
            return sendRelayControlOn();
            case IDC_INTERRUPT_PIN:
            return sendInterruptPin();
        case IDC_SWITCH_COUNTING_SETTINGS:
            return sendSwitchCountingSettings();
#endif
        case IDC_VERSION:
            sendStartResponse(IDC_VERSION);
#ifdef MEM_32KB
            sendSerial((uint8_t)2);
#else
            sendSerial((uint8_t)1);
#endif
            return OK;
        case IDC_CURRENT_TIME:
            sendStartResponse(IDC_CURRENT_TIME);
            sendSerial(RelayController::getRemoteTimeSec());
            return OK;
#ifdef MEM_32KB
        case IDC_FIX_DATA:
            return sendFixData();
        case IDC_SWITCH_DATA:
            return sendSwitchData();
        case IDC_CONTACT_WAIT_DATA:
            return sendContactWaitData();
#endif
        case IDC_GET_CYCLES_STATISTICS:
            sendStartResponse(IDC_GET_CYCLES_STATISTICS);
            sendSerial(minCycleDuration);
            sendSerial(maxCycleDuration);
            sendSerial((uint16_t)(millis() / cyclesCount));
            sendSerial(cyclesCount);
            return OK;
        default:
            return E_UNDEFINED_OPERATION;
    }
}

ErrorCode Server::processBinarySet(InstructionDataCode code) {
    switch (code) {
        case IDC_SETTINGS:
            return (ErrorCode) saveSettings();
        case IDC_STATE:
            return saveState();
        case IDC_ID:
            return saveId();
        case IDC_STATE_FIX_SETTINGS:
            return saveStateFixSettings();
        case IDC_REMOTE_TIMESTAMP:
            return saveRemoteTimestamp();
        case IDC_RELAY_STATE:
            return saveRelayState();
#ifdef MEM_32KB
        case IDC_ALL:
            return saveAll();
        case IDC_RELAY_DISABLED_TEMP:
            return saveRelayDisabledTemp();
        case IDC_RELAY_SWITCHED_ON:
            return saveRelaySwitchedOn();
        case IDC_INTERRUPT_PIN:
            return saveInterruptPin();
        case IDC_SWITCH_COUNTING_SETTINGS:
            return saveSwitchCountingSettings();
        case IDC_CLEAR_SWITCH_COUNT:
            return clearSwitchCount();
#endif
        default:
            return E_UNDEFINED_OPERATION;
    }
}

void Server::sendSettings(bool addResultCode) {
    uint8_t relayCount = settings.getRelaysCount();
    if (addResultCode) {
        sendStartResponse(IDC_SETTINGS);
        sendSerial(relayCount);
    }
    for (uint8_t i = 0; i < relayCount; i++) {
        const RelaySettings &relay = settings.getRelaySettingsRef(i);
        sendSerial(relay.getSetPinSettings().getRaw());
        sendSerial(relay.getMonitorPinSettings().getRaw());
        sendSerial(relay.getControlPinSettings().getRaw());
    }
}

uint8_t Server::saveSettings() {
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
            return E_RELAY_NOT_ALLOWED_PIN_USED | relaySettings[i].getSetPinSettings().getPin();
        }
        if (!relaySettings[i].getMonitorPinSettings().isAllowedPin()){
            return E_RELAY_NOT_ALLOWED_PIN_USED | relaySettings[i].getMonitorPinSettings().getPin();
        }
        if (!relaySettings[i].getControlPinSettings().isAllowedPin()){
            return E_RELAY_NOT_ALLOWED_PIN_USED | relaySettings[i].getControlPinSettings().getPin();
        }
    }
    uint8_t savedCount = settings.saveRelaySettings(relaySettings, relayCount);
    return savedCount | E_UNDEFINED_CODE;
}

ErrorCode Server::sendState(bool addResultCode) {
    uint8_t count = settings.getRelaysCount();
    uint8_t pairsCount = count / 2 + count % 2;
    uint8_t result[pairsCount];
    for (uint8_t i = 0; i < count; i += 2) {
        uint8_t tmpResult1 = readRelayStateBits(i);
        uint8_t tmpResult2 = i + 1 < count ? readRelayStateBits(i + 1) : 0;
        result[i / 2] = tmpResult2 << 4 | tmpResult1;
    }
    if (addResultCode) {
        sendStartResponse(IDC_STATE);
        sendSerial(count);
    }
    for (uint8_t i = 0; i < pairsCount; i++) {
        sendSerial(result[i]);
    }
    return OK;
}

ErrorCode Server::saveState() {
    uint8_t providedCount = 0;
    ErrorCode res = readRelayCountFromCmdBuff(providedCount);
    if (res != OK) return res;
    if (providedCount != settings.getRelaysCount()) return E_RELAY_COUNT_AND_DATA_MISMATCH;
    if (cmdBuffSize < cmdBuffCurrPos + SET_RELAY_STATE_DATA_SIZE * providedCount + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t* data = cmdBuff + cmdBuffCurrPos;
    for (uint8_t i = 0; i < providedCount; i++) {
        uint8_t command = data[i / SET_RELAY_STATE_DATA_COUNT_IN_BYTE];
        uint8_t shift = (i % SET_RELAY_STATE_DATA_COUNT_IN_BYTE) * SET_RELAY_STATE_DATA_SIZE;
        RelayController::setRelayState(i, CHECK_BIT(command, shift));
        RelayController::setControlTemporaryDisabled(i, CHECK_BIT(command, shift + 1));
    }
    return OK;
}

ErrorCode Server::sendId(bool addResultCode) {
    if (addResultCode) {
        sendStartResponse(IDC_ID);
    }
    sendSerial(settings.getControllerId());
    return OK;
}

ErrorCode Server::saveId() {
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint32_t)) return E_REQUEST_DATA_NO_VALUE;
    uint32_t controllerId = 0;
    ErrorCode res = readUint32FromCommandBuffer(controllerId);
    if (res != OK) return res;
    settings.saveControllerId(controllerId);
    return OK;
}

ErrorCode Server::sendStateFixSettings() {
    sendStartResponse(IDC_STATE_FIX_SETTINGS);
    const StateFixSettings &stateFixSettings = settings.getStateFixSettings();
    sendSerial(stateFixSettings.getDelayMillis());
    sendSerial(stateFixSettings.getMaxCount());
    sendSerial(stateFixSettings.getMinWaitDelaySec());
    sendSerial(stateFixSettings.getContactReadyWaitDelayMillis());
    return OK;
}

ErrorCode Server::saveStateFixSettings() {
    uint16_t delayMillis = 0;
    ErrorCode res = readUint16FromCmdBuff(delayMillis);
    if (res != OK) return res;
    uint8_t maxCount = 0;
    res = readUint8FromCmdBuff(maxCount);
    if (res != OK) return res;
    uint8_t minWaitDelaySec = 0;
    res = readUint8FromCmdBuff(minWaitDelaySec);
    if (res != OK) return res;
    uint16_t contactReadyWaitDelayMillis = 0;
    res = readUint16FromCmdBuff(contactReadyWaitDelayMillis);
    if (res != OK) return res;
    settings.saveStateFixSettings(StateFixSettings(delayMillis, maxCount, minWaitDelaySec, contactReadyWaitDelayMillis));
    return OK;
}

ErrorCode Server::sendRemoteTimestamp() {
    sendStartResponse(IDC_REMOTE_TIMESTAMP);
    sendSerial(RelayController::getRemoteTimeStamp());
    return OK;
}

ErrorCode Server::saveRemoteTimestamp() {
    uint32_t value = 0;
    ErrorCode res = readUint32FromCommandBuffer(value);
    if (res != OK) return res;
    RelayController::setRemoteTimeStamp(value);
    return OK;
}

#ifdef MEM_32KB

ErrorCode Server::sendInterruptPin(bool addResultCode) {
    if (addResultCode) {
        sendStartResponse(IDC_INTERRUPT_PIN);
    }
    sendSerial(settings.getControlInterruptPin());
    return OK;
}

ErrorCode Server::saveInterruptPin() {
    uint8_t pin = 0;
    ErrorCode res = readUint8FromCmdBuff(pin);
    if (res != OK) return res;
    return settings.saveControlInterruptPin(pin) ? OK : E_CONTROL_INTERRUPTED_PIN_NOT_ALLOWED_VALUE;
}

ErrorCode Server::sendSwitchCountingSettings() {
    sendStartResponse(IDC_SWITCH_COUNTING_SETTINGS);
    sendSerial(settings.getSwitchCountingSettingsRef().getSwitchLimitIntervalSec());
    sendSerial(settings.getSwitchCountingSettingsRef().getMaxSwitchCount());
    return OK;
}

ErrorCode Server::saveSwitchCountingSettings() {
    uint8_t relayIdx = 0;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIdx);
    if (res != OK) return res;
    uint16_t switchLimitIntervalSec = 0;
    res = readUint16FromCmdBuff(switchLimitIntervalSec);
    if (res != OK) return res;
    uint8_t switchMaxCount = 0;
    res = readUint8FromCmdBuff(switchMaxCount);
    if (res != OK) return res;
    if (switchMaxCount > MAX_SWITCH_LIMIT_COUNT) return E_SWITCH_COUNT_MAX_VALUE_OVERFLOW;
    settings.saveSwitchCountingSettings(SwitchCountingSettings(switchLimitIntervalSec, switchMaxCount));
    return OK;
}

ErrorCode Server::clearSwitchCount() {
    uint8_t relayIdx = 0;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIdx);
    if (res != OK) return res;
    RelayController::clearSwitchCount(relayIdx);
    return OK;
}

ErrorCode Server::sendAll() {
    sendStartResponse(IDC_ALL);
    sendId(false);
    sendInterruptPin(false);
    sendSerial(settings.getRelaysCount());
    sendSettings(false);
    sendState(false);
    return OK;
}

ErrorCode Server::saveAll() {
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
ErrorCode Server::sendFixData() {
    sendStartResponse(IDC_FIX_DATA);
    uint8_t count = settings.getRelaysCount();
    sendSerial(count);
    for (uint8_t i = 0; i < count; i++) {
        sendSerial(RelayController::getFixTryCount(i));
        sendSerial(RelayController::getFixLastTryTime(i));
    }
    return OK;
}

ErrorCode Server::sendSwitchData() {
    sendStartResponse(IDC_SWITCH_DATA);
    uint32_t *switchData;
    uint8_t dataCount = RelayController::getSwitchData(&switchData);
    sendSerial(dataCount);
    for (uint8_t i = 0; i < dataCount; i++) {
        sendSerial(switchData[i]);
    }
    return OK;
}

ErrorCode Server::sendContactWaitData() {
    sendStartResponse(IDC_CONTACT_WAIT_DATA);
    for (uint8_t i = 0; i < settings.getRelaysCount(); i++) {
        sendSerial(RelayController::getContactStartWait(i));
    }
    return OK;
}

ErrorCode Server::sendRelayDisabledTemp() {
    return send([]  (Server *communicator, uint8_t relayIndex) {
        uint8_t value = RelayController::isControlTemporaryDisabled(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_DISABLED_TEMP);
}

ErrorCode Server::saveRelayDisabledTemp() {
    return save([] (Server *communicator, uint8_t relayIndex, uint8_t cmdData) {
        RelayController::setControlTemporaryDisabled(relayIndex, CHECK_BIT(cmdData, 0));
    });
}

ErrorCode Server::sendRelaySwitchedOn() {
    return send([]  (Server *communicator, uint8_t relayIndex) {
        uint8_t value = RelayController::getRelayLastState(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_SWITCHED_ON);
}

ErrorCode Server::saveRelaySwitchedOn() {
    return save([] (Server *communicator, uint8_t relayIndex, uint8_t cmdData) {
        RelayController::setRelayState(relayIndex, CHECK_BIT(cmdData, 0));
    });
}

ErrorCode Server::sendRelayMonitorOn() {
    return send([]  (Server *communicator, uint8_t relayIndex) {
        uint8_t value = RelayController::checkRelayMonitoringState(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_MONITOR_ON);
}

ErrorCode Server::sendRelayControlOn() {
    return send([]  (Server *communicator, uint8_t relayIndex) {
        uint8_t value = RelayController::checkControlPinState(relayIndex);
        value = (value << 4) | relayIndex;
        return value;
    }, IDC_RELAY_CONTROL_ON);
}



ErrorCode Server::send(uint8_t(*getter)(Server*, uint8_t), InstructionDataCode dataCode) {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(relayIndex);
    if (res != OK) return res;
    uint8_t value = getter(this, relayIndex);
    sendStartResponse(dataCode);
    if (value <= 0x0f) {
        sendSerial((uint8_t)((relayIndex & 0x0F) | (value << 4)));
    } else {
        sendSerial(relayIndex);
        sendSerial(value);
    }
    return OK;
}

ErrorCode Server::save(void(*setter)(Server*, uint8_t, uint8_t)) {
    uint8_t cmdData = 0;
    ErrorCode res = readUint8FromCmdBuff(cmdData);
    if (res != OK) return res;
    uint8_t relayIndex = cmdData & 0x0f;
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = cmdData >> 4;
    setter(this, relayIndex, value);
    return OK;
}

ErrorCode Server::sendRelayState() {
    return send([]  (Server *communicator, uint8_t relayIndex) {
        return Server::readRelayStateBits(relayIndex);
    }, IDC_RELAY_STATE);
}

ErrorCode Server::saveRelayState() {
    return save([] (Server *communicator, uint8_t relayIndex, uint8_t cmdData) {
        RelayController::setRelayState(relayIndex, CHECK_BIT(cmdData, 0));
        RelayController::setControlTemporaryDisabled(relayIndex, CHECK_BIT(cmdData, 1));
    });
}

#else

ErrorCode Server::sendRelayState() {
    uint8_t relayIndex;
    ErrorCode res = readUint8FromCmdBuff(relayIndex);
    if (res != OK) return res;
    if (res >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = Server::readRelayStateBits(relayIndex);
    sendStartResponse(IDC_RELAY_STATE);
    sendSerial((uint8_t)((relayIndex & 0x0F) | (value << 4)));
    return OK;
}

ErrorCode Server::saveRelayState() {
    uint8_t cmdData = 0;
    ErrorCode res = readUint8FromCmdBuff(cmdData);
    if (res != OK) return res;
    uint8_t relayIndex = cmdData & 0x0f;
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    RelayController::setRelayState(relayIndex, CHECK_BIT(cmdData, 4));
    RelayController::setControlTemporaryDisabled(relayIndex, CHECK_BIT(cmdData, 4));
    return OK;
}

#endif

ErrorCode Server::readUint8FromCmdBuff(uint8_t &result) {
    if (cmdBuffSize < cmdBuffCurrPos + 1) return E_REQUEST_DATA_NO_VALUE;
    result = cmdBuff[cmdBuffCurrPos++];
    return OK;
}

ErrorCode Server::readUint16FromCmdBuff(uint16_t &result) {
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint16_t)) return E_REQUEST_DATA_NO_VALUE;
    result |= ((uint_fast16_t)cmdBuff[cmdBuffCurrPos++]) << 8;
    result |= cmdBuff[cmdBuffCurrPos++];
    return OK;
}

ErrorCode Server::readUint32FromCommandBuffer(uint32_t &result) {
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint32_t)) return E_REQUEST_DATA_NO_VALUE;
    result |= ((uint_fast32_t)cmdBuff[cmdBuffCurrPos++]) << 24;
    result |= ((uint_fast32_t)cmdBuff[cmdBuffCurrPos++]) << 16;
    result |= ((uint_fast32_t)cmdBuff[cmdBuffCurrPos++]) << 8;
    result |= cmdBuff[cmdBuffCurrPos++];
    return OK;
}

#ifdef MEM_32KB
ErrorCode Server::readRelayIndexFromCmdBuff(uint8_t &result) {
    uint8_t res = 0;
    ErrorCode readRes = readUint8FromCmdBuff(res);
    if (readRes != OK) return readRes;
    if (res >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    result = res;
    return OK;
}
#endif

ErrorCode Server::readRelayCountFromCmdBuff(uint8_t &count) {
    uint8_t res = 0;
    ErrorCode readRes = readUint8FromCmdBuff(res);
    if (readRes != OK) return readRes;
    if (res >= MAX_RELAYS_COUNT) return E_RELAY_COUNT_OVERFLOW;
    count = res;
    return OK;
}

uint8_t Server::readRelayStateBits(uint8_t relayIndex) {
    uint8_t result = 0;
    setBit(result, 0, RelayController::checkRelayMonitoringState(relayIndex));
    setBit(result, 1, RelayController::getRelayLastState(relayIndex));
    setBit(result, 2, RelayController::isControlTemporaryDisabled(relayIndex));
    setBit(result, 3, RelayController::checkControlPinState(relayIndex));
    return result;
}

