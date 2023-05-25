//
// Created by valti on 24.05.2023.
//

#ifndef RELAYCONTROLLER_COMMUNICATIONPROTOCOL_H
#define RELAYCONTROLLER_COMMUNICATIONPROTOCOL_H


enum InstructionCode {
    IC_NONE = 0,
    IC_READ = 1,
    IC_SET = 2,
    IC_SUCCESS = 3,
    IC_ERROR = 4,
    IC_EXECUTE = 5,
    IC_REQUEST = 6,
    IC_RESPONSE = 7,
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
    IDC_INTERRUPT_PIN = 10,
    IDC_CONTACT_READY_WAIT_DELAY = 11,
    IDC_SWITCH_COUNT_INTERVAL_SEC = 12,
    IDC_STATE_FIX_SETTINGS = 13,
    IDC_RELAY_STATE_CHANGED = 14,
    IDC_REMOTE_TIMESTAMP = 15,
    IDC_VERSION = 16,
    IDC_FIX_DATA = 17,
    IDC_SWITCH_DATA = 18,
    IDC_GET_TIME_STAMP = 19,
    IDC_CONTACT_WAIT_DATA = 20,
#ifdef MEM_32KB
    IDC_MAX_SWITCH_COUNT = 13,
    IDC_CLEAR_SWITCH_COUNT = 14,
#endif
    IDC_UNKNOWN = 16
};

enum ErrorCode {
    OK = 0,
    E_REQUEST_DATA_NO_VALUE = 1,
    E_INSTRUCTION_UNRECOGIZED = 2,
    E_COMMAND_EMPTY = 3,
    E_COMMAND_SIZE_OVERFLOW = 4,
    E_INSTRUCTION_WRONG_START = 5,
    E_WRITE_MAX_ATTEMPTS_EXCEDED = 6,
    E_UNDEFINED_OPERATION = 7,
    E_RELAY_COUNT_OVERFLOW = 8,
    E_RELAY_COUNT_AND_DATA_MISMATCH = 9,
    E_RELAY_INDEX_OUT_OF_RANGE = 10,
    E_CONTROL_INTERRUPTED_PIN_NOT_ALLOWED_VALUE = 11,
    E_RELAY_NOT_ALLOWED_PIN_USED = 0b00100000,
    E_UNDEFINED_CODE = 128
};

static const int MAX_COMMAND_READ_TIME = 50;


inline size_t sendSerial(bool value, Stream &serial = Serial) {
    return serial.write(value ? 1 : 0);
}

inline size_t sendSerial(uint8_t value, Stream &serial = Serial) {
    return serial.write(value);
}

inline size_t sendSerial(InstructionCode value, Stream &serial = Serial) {
    return sendSerial((uint8_t)value, serial);
}

inline size_t sendSerial(InstructionDataCode value, Stream &serial = Serial) {
    return sendSerial((uint8_t)value, serial);
}

inline size_t sendSerial(ErrorCode buffer, Stream &serial = Serial) {
    return serial.write(buffer);
}

inline size_t sendSerial(uint16_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    sum += serial.write(value >> 8);
    sum += serial.write(value & 0xFF);
    return sum;
}

inline size_t sendSerial(int32_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    uint8_t shift = 24;
    for (uint8_t i = 0; i < 3; i++) {
        sum += serial.write((uint8_t) ((value >> shift) & 0xFF) );
        shift -= 8;
    }
    sum += serial.write(value & 0xFF);
    return sum;
}

inline size_t sendSerial(uint32_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    uint8_t shift = 24;
    for (uint8_t i = 0; i < 3; i++) {
        sum += serial.write((uint8_t) ((value >> shift) & 0xFF) );
        shift -= 8;
    }
    sum += serial.write(value & 0xFF);
    return sum;
}

inline size_t sendSerial(uint64_t value, Stream &serial = Serial) {
    uint8_t sum = 0;
    uint8_t shift = 56;
    for (uint8_t i = 0; i < 7; i++) {
        sum += serial.write((uint8_t) ((value >> shift) & 0xFF) );
        shift -= 8;
    }
    sum += serial.write((uint8_t)(value & 0xFF));
    return sum;
}

inline size_t sendSerial(const uint8_t *buffer, size_t size, Stream &serial = Serial) {
    return serial.write(buffer, size);
}

inline size_t sendSerial(const char *buffer, Stream &serial = Serial) {
    return serial.write(buffer);
}

inline void sendStartResponse(InstructionDataCode code) {
    sendSerial(IC_NONE);
    sendSerial(IC_RESPONSE);
    sendSerial(code);
}

inline void sendStartRequest(InstructionDataCode code) {
    sendSerial(IC_NONE);
    sendSerial(IC_REQUEST);
    sendSerial(code);
}

#endif //RELAYCONTROLLER_COMMUNICATIONPROTOCOL_H
