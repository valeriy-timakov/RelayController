//
// Created by valti on 06.05.2023.
//

#include "Communicator.h"
#include "utils.h"

#define MAIN_CODE_POSITION 0
#define INSTRUCTION_CODE_POSITION MAIN_CODE_POSITION + 1
#define INSTRUCTION_DATA_START_CODE_POSITION INSTRUCTION_CODE_POSITION + 1
#define SETTINGS_SIZE_PER_RELAY 3
#define SET_RELAY_STATE_DATA_SIZE 2
#define SET_RELAY_STATE_DATA_COUNT_IN_BYTE (sizeof (uint8_t) / SET_RELAY_STATE_DATA_SIZE)


enum InstructionCode {
    IC_NONE = 0,
    IC_READ = 1,
    IC_SET = 2,
    IC_SUCCESS = 3,
    IC_ERROR = 4,
    IC_UNKNOWN = 16
};

enum InstructionDataCode {
    IDC_NONE = 0,
    IDC_SETTINGS = 1,
    IDC_STATE = 2,
    IDC_ID = 3,
    IDC_ALL = 4,
    IDC_RELAY_STATE = 5,
    IDC_RELAY_DISABLED_TEMP = 6,
    IDC_RELAY_SWITCHED_ON = 7,
    IDC_RELAY_MONITOR_ON = 8,
    IDC_RELAY_CONTROL_ON = 9,
    IDC_UNKNOWN = 16
};


void Communicator::setup() {
    Serial.begin(9600);
    Serial.println("...");
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
    sum += serial.write(value >> 24);
    sum += serial.write((value >> 16) & 0xFF);
    sum += serial.write((value >> 8) & 0xFF);
    sum += serial.write(value & 0xFF);
    return sum;
}

size_t sendSerial(uint64_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    sum += serial.write((uint8_t)(value >> 56));
    sum += serial.write((uint8_t)((value >> 48) & 0xFF));
    sum += serial.write((uint8_t)((value >> 40) & 0xFF));
    sum += serial.write((uint8_t)((value >> 32) & 0xFF));
    sum += serial.write((uint8_t)((value >> 24) & 0xFF));
    sum += serial.write((uint8_t)((value >> 16) & 0xFF));
    sum += serial.write((uint8_t)((value >> 8) & 0xFF));
    sum += serial.write((uint8_t)(value & 0xFF));
    return sum;
}

size_t sendSerial(const uint8_t *buffer, size_t size, Stream &serial = Serial) {
    return serial.write(buffer, size);
}

size_t sendSerial(const char *buffer, Stream &serial = Serial) {
    return serial.write(buffer);
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

bool Communicator::readBinaryCommand() {
    uint8_t available = Serial.available();
    if (!available) {
        return false;
    }
    Serial.println("readBinaryCommand");
    if (commandParsed && !commandPocessed) {
        return true;
    }
    uint32_t currTime = (uint32_t) millis();
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
        Serial.write(cmdBuff, cmdBuffSize);
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
        sendError(result);
    }
    commandPocessed = true;
}

ErrorCode Communicator::processBinaryRead() {
    InstructionDataCode code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {
        case IDC_SETTINGS:
            sendSettings(true);
            return OK;
        case IDC_STATE:
            return sendState(true);
        case IDC_ID:
            return sendId(true);
        case IDC_ALL:
            return sendAll(true);
        case IDC_RELAY_STATE:
            return sendRelayState(true);
        case IDC_RELAY_DISABLED_TEMP:
            return sendRelayDisabledTemp(true);
        case IDC_RELAY_SWITCHED_ON:
            return sendRelaySwitchedOn(true);
        case IDC_RELAY_MONITOR_ON:
            return sendRelayMonitorOn(true);
        case IDC_RELAY_CONTROL_ON:
            return sendRelayControlOn(true);
    }
    return E_UNDEFINED_OPERATION;
}

ErrorCode Communicator::processBinarySet() {
    InstructionDataCode code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {
        case IDC_SETTINGS:
            return saveSettings();
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

ErrorCode Communicator::saveSettings() {
    if (cmdBuffSize < cmdBuffCurrPos + 1) {
        return E_REQUEST_DATA_NO_VALUE;
    }
    uint8_t relayCount = cmdBuff[cmdBuffCurrPos++];
    if (relayCount > MAX_RELAYS_COUNT) {
        return E_RELAY_COUNT_OVERFLOW;
    }
    if (relayCount * SETTINGS_SIZE_PER_RELAY + cmdBuffCurrPos > cmdBuffSize) {
        return E_RELAY_COUNT_AND_DATA_MISMATCH;
    }
    settings.saveRelaysCount(relayCount);
    for (uint8_t i = 0; i < relayCount; i++) {
        settings.saveRelaySettings(i,
            cmdBuff[cmdBuffCurrPos++],
            cmdBuff[cmdBuffCurrPos++],
            cmdBuff[cmdBuffCurrPos++]);
    }
    return OK;
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
    uint8_t count = settings.getRelaysCount();
    if (cmdBuffSize < cmdBuffCurrPos + SET_RELAY_STATE_DATA_SIZE * count + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t providedCount = cmdBuff[cmdBuffCurrPos++];
    if (providedCount != count) return E_RELAY_COUNT_AND_DATA_MISMATCH;
    uint8_t* data = cmdBuff + cmdBuffCurrPos;
    for (uint8_t i = 0; i < count; i++) {
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
    if (cmdBuffSize < cmdBuffCurrPos + sizeof(uint32_t)) {
        return E_REQUEST_DATA_NO_VALUE;
    }
    uint32_t controllerId = *( (uint32_t*)( cmdBuff + cmdBuffCurrPos ) );
    cmdBuffCurrPos += sizeof(uint32_t);
    settings.saveControllerId(controllerId);
    return OK;
}

ErrorCode Communicator::sendAll(bool addResultCode) {
    if (addResultCode) {
        sendStartAnswer(IDC_ALL);
    }
    sendId();
    sendSerial(settings.getRelaysCount());
    sendSettings();
    sendState();
    return OK;
}

ErrorCode Communicator::saveAll() {
    if (cmdBuffSize < cmdBuffCurrPos + 1) {
        return E_REQUEST_DATA_NO_VALUE;
    }
    uint8_t relayCount = cmdBuff[cmdBuffCurrPos++];
    if (relayCount > MAX_RELAYS_COUNT) {
        return E_RELAY_COUNT_OVERFLOW;
    }
    if (cmdBuffCurrPos + sizeof(uint32_t) + relayCount * SETTINGS_SIZE_PER_RELAY + SET_RELAY_STATE_DATA_SIZE * relayCount + 1 > cmdBuffSize) {
        return E_REQUEST_DATA_NO_VALUE;
    }

    saveId();
    saveSettings();
    saveState();

    return OK;
}

ErrorCode Communicator::sendRelayState(bool addResultCode) {
    if (cmdBuffSize < cmdBuffCurrPos + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t relayIndex = cmdBuff[cmdBuffCurrPos++];
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = readRelayStateBits(relayIndex);
    if (addResultCode) {
        sendStartAnswer(IDC_RELAY_STATE);
    }
    sendSerial(value);
    return OK;
}

ErrorCode Communicator::saveRelayState() {
    if (cmdBuffSize < cmdBuffCurrPos + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t command = cmdBuff[cmdBuffCurrPos++];
    uint8_t relayIndex = (command & 0xF0) >> 4;
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = command & 0x0F;
    relayController.setRelayState(relayIndex, CHECK_BIT(value, 0));
    relayController.setControlTemporaryDisabled(relayIndex, CHECK_BIT(value, 1));
    return OK;
}

ErrorCode Communicator::sendRelayDisabledTemp(bool addResultCode) {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(&relayIndex);
    if (res != OK) return res;
    bool disabled = relayController.isControlTemporaryDisabled(relayIndex);
    if (addResultCode) {
        sendStartAnswer(IDC_RELAY_DISABLED_TEMP);
    }
    sendSerial(relayIndex);
    sendSerial(disabled);
    return OK;
}

ErrorCode Communicator::saveRelayDisabledTemp() {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(&relayIndex);
    if (res != OK) return res;
    uint8_t disabled = 0;
    res = readUint8FromCmdBuff(&disabled);
    if (res != OK) return res;
    relayController.setControlTemporaryDisabled(relayIndex, disabled);
    return OK;
}

ErrorCode Communicator::sendRelaySwitchedOn(bool addResultCode) {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(&relayIndex);
    if (res != OK) return res;
    if (addResultCode) {
        sendStartAnswer(IDC_RELAY_SWITCHED_ON);
    }
    sendSerial(relayController.getRelayLastState(relayIndex));
    return OK;
}

ErrorCode Communicator::saveRelaySwitchedOn() {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(&relayIndex);
    if (res != OK) return res;
    uint8_t switchedOn = 0;
    res = readUint8FromCmdBuff(&switchedOn);
    if (res != OK) return res;
    relayController.setRelayState(relayIndex, switchedOn);
    return OK;
}

ErrorCode Communicator::sendRelayMonitorOn(bool addResultCode) {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(&relayIndex);
    if (res != OK) return res;
    if (addResultCode) {
        sendStartAnswer(IDC_RELAY_SWITCHED_ON);
    }
    sendSerial(relayController.checkRelayMonitoringState(relayIndex));
    return OK;
}

ErrorCode Communicator::sendRelayControlOn(bool addResultCode) {
    uint8_t relayIndex;
    ErrorCode res = readRelayIndexFromCmdBuff(&relayIndex);
    if (res != OK) return res;
    if (addResultCode) {
        sendStartAnswer(IDC_RELAY_SWITCHED_ON);
    }
    sendSerial(relayController.checkControlPinState(relayIndex));
    return OK;
}

ErrorCode Communicator::readUint8FromCmdBuff(uint8_t *result) {
    if (cmdBuffSize < cmdBuffCurrPos + 1) return E_REQUEST_DATA_NO_VALUE;
    *result = cmdBuff[cmdBuffCurrPos++];
    return OK;
}

ErrorCode Communicator::readRelayIndexFromCmdBuff(uint8_t *result) {
    uint8_t res = 0;
    ErrorCode readRes = readUint8FromCmdBuff(&res);
    if (readRes != OK) return readRes;
    if (res >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    *result = res;
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


