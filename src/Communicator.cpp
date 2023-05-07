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
    IC_EXECUTE = 3,
    IC_SUCCESS = 5,
    IC_ERROR = 6,
    IC_REBOOT = 7,
    IC_UNKNOWN = 16
};

enum InstructionDataCode {
    IDC_NONE = 0,
    IDC_SETTINGS = 1,
    IDC_STATE = 2,
    IDC_RELAY_STATE = 3,
    IDC_UNKNOWN = 16
};


void Communicator::setup() {
    Serial.begin(9600);
    Serial.print("Starting...");
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
    Serial.print("Settings available: " );
    Serial.println(available);
    Serial.print(lastPacketSize);
    Serial.print("/" );
    Serial.println(lastPacketTime);
    uint32_t currTime = (uint32_t) millis();
    bool commandReady = lastPacketSize > 0 && ( (available - lastPacketSize) == 0 || (currTime - lastPacketTime) > MAX_COMMAND_READ_TIME );
    lastPacketSize = available;
    lastPacketTime = currTime;
    Serial.print("a: " );
    Serial.print(lastPacketSize);
    Serial.print("/" );
    Serial.println(lastPacketTime);
    if (!commandReady) {
        return false;
    }
    lastPacketSize = 0;
    Serial.println("Command ready");
    int r = Serial.read();
    if (r != IC_NONE) {
        Serial.println(r);
        clearSerial();
        sendError(E_INSTRUCTION_WRONG_START);
        return false;
    }
    curCmdBuffPos = 0;
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
    curCmdBuffPos += available;
    bool commandUnrecognized = false;
    if (curCmdBuffPos < 2) {
        commandUnrecognized = true;
    } else {
        uint8_t firstCode = cmdBuff[MAIN_CODE_POSITION];
        commandUnrecognized = firstCode != IC_READ && firstCode != IC_SET && firstCode != IC_EXECUTE;
    }
    if (commandUnrecognized) {
        sendError(E_INSTRUCTION_UNRECOGIZED);
        return false;
    }
    Serial.println("Command processed");
    commandParsed = true;
    return true;
}

void Communicator::processBinaryInstruction() {
    uint8_t mainCode = cmdBuff[MAIN_CODE_POSITION];
    ErrorCode result;
    switch (mainCode) {
        case IC_READ:
            result = processBinaryRead();
            break;
        case IC_SET:
            if (curCmdBuffPos < 3) {
                result = E_REQUEST_DATA_NO_VALUE;
                break;
            }
            result = processBinarySet();
            if (result == OK) {
                sendSuccess();
            }
            break;
        case IC_EXECUTE:
            result = processBinaryExecute();
            if (result == OK) {
                sendSuccess();
            }
            break;
    }
    if (result != OK) {
        sendError(result);
    }
}

ErrorCode Communicator::processBinaryRead() {
    InstructionDataCode code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {
        case IDC_SETTINGS:
            sendSettings();
            return OK;
        case IDC_RELAY_STATE:
            return sendRelayState();
        case IDC_STATE:
            return sendState();
    }
    return E_UNDEFINED_OPERATION;
}

ErrorCode Communicator::processBinarySet() {
    InstructionDataCode code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {
        case IDC_SETTINGS:
            return saveSettings();
            break;
        case IDC_STATE:
            return saveState();
        case IDC_RELAY_STATE:
            return saveRelayState();
    }
    return E_UNDEFINED_OPERATION;
}

ErrorCode Communicator::processBinaryExecute() {
    InstructionDataCode code = (InstructionDataCode) cmdBuff[INSTRUCTION_CODE_POSITION];
    switch (code) {

    }
    return E_UNDEFINED_OPERATION;
}

void Communicator::sendSettings() {
    uint8_t relayCount = settings.getRelaysCount();
    sendSerial(IC_NONE);
    sendSerial(IDC_SETTINGS);
    sendSerial(settings.getControllerId());
    sendSerial(relayCount);
    for (uint8_t i = 0; i < relayCount; i++) {
        const RelaySettings &relay = settings.getRelaySettingsRef(i);
        sendSerial(relay.getSetPinSettings().getRaw());
        sendSerial(relay.getMonitorPinSettings().getRaw());
        sendSerial(relay.getControlPinSettings().getRaw());
    }
}

ErrorCode Communicator::saveSettings() {
    uint8_t pos = INSTRUCTION_DATA_START_CODE_POSITION;
    if (curCmdBuffPos < pos + sizeof(uint32_t) + 1) {
        return E_REQUEST_DATA_NO_VALUE;
    }
    uint32_t controllerId = *( (uint32_t*)( cmdBuff + pos ) );
    pos += sizeof controllerId;
    uint8_t relayCount = cmdBuff[pos++];
    if (relayCount > MAX_RELAYS_COUNT) {
        return E_RELAY_COUNT_OVERFLOW;
    }
    if (relayCount * SETTINGS_SIZE_PER_RELAY + pos > curCmdBuffPos) {
        return E_RELAY_COUNT_AND_DATA_MISMATCH;
    }
    settings.saveControllerId(controllerId);
    settings.saveRelaysCount(relayCount);
    for (uint8_t i = 0; i < relayCount; i++) {
        settings.saveRelaySettings(i, cmdBuff[pos++], cmdBuff[pos++], cmdBuff[pos++]);
    }
    return OK;
}

ErrorCode Communicator::sendRelayState() {
    if (curCmdBuffPos < INSTRUCTION_DATA_START_CODE_POSITION + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t relayIndex = cmdBuff[INSTRUCTION_DATA_START_CODE_POSITION];
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = readRelayStateBits(relayIndex);
    sendSerial(IC_NONE);
    sendSerial(IDC_RELAY_STATE);
    sendSerial(value);
    return OK;
}

ErrorCode Communicator::saveRelayState() {
    if (curCmdBuffPos < INSTRUCTION_DATA_START_CODE_POSITION + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t command = cmdBuff[INSTRUCTION_DATA_START_CODE_POSITION];
    uint8_t relayIndex = (command & 0xF0) >> 4;
    if (relayIndex >= settings.getRelaysCount()) return E_RELAY_INDEX_OUT_OF_RANGE;
    uint8_t value = command & 0x0F;
    relayController.setRelayState(relayIndex, CHECK_BIT(value, 0));
    relayController.setControlTemporaryDisabled(relayIndex, CHECK_BIT(value, 1));
    return OK;
}



ErrorCode Communicator::sendState() {
    uint8_t count = settings.getRelaysCount();
    uint8_t pairsCount = count / 2 + count % 2;
    uint8_t result[pairsCount];
    for (uint8_t i = 0; i < count; i += 2) {
        uint8_t tmpResult1 = readRelayStateBits(i);
        uint8_t tmpResult2 = i + 1 < count ? readRelayStateBits(i + 1) : 0;
        result[i / 2] = readRelayStateBits(tmpResult2 << 4 | tmpResult1);
    }
    sendSerial(IC_NONE);
    sendSerial(IDC_STATE);
    sendSerial(count);
    for (uint8_t i = 0; i < pairsCount; i++) {
        sendSerial(result[i]);
    }
    return OK;
}

ErrorCode Communicator::saveState() {
    uint8_t count = settings.getRelaysCount();
    if (curCmdBuffPos < INSTRUCTION_DATA_START_CODE_POSITION + SET_RELAY_STATE_DATA_SIZE * count + 1) return E_REQUEST_DATA_NO_VALUE;
    uint8_t providedCount = cmdBuff[INSTRUCTION_DATA_START_CODE_POSITION];
    if (providedCount != count) return E_RELAY_COUNT_AND_DATA_MISMATCH;
    uint8_t* data = cmdBuff + INSTRUCTION_DATA_START_CODE_POSITION + 1;
    const uint8_t pagesCount = count / SET_RELAY_STATE_DATA_COUNT_IN_BYTE + ((count % SET_RELAY_STATE_DATA_COUNT_IN_BYTE) > 0 ? 1 : 0);
    for (uint8_t i = 0; i < pagesCount; i++) {
        uint8_t command = data[i];
        relayController.setRelayState(i * SET_RELAY_STATE_DATA_COUNT_IN_BYTE, CHECK_BIT(command, 0));
        relayController.setControlTemporaryDisabled(i * SET_RELAY_STATE_DATA_COUNT_IN_BYTE, CHECK_BIT(command, 1));
    }
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
