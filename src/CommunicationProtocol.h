//
// Created by valti on 24.05.2023.
//

#ifndef RELAYCONTROLLER_COMMUNICATIONPROTOCOL_H
#define RELAYCONTROLLER_COMMUNICATIONPROTOCOL_H


enum InstructionCode {
    IC_NONE = 0x00,
    IC_READ = 0x01,
    IC_SET = 0x02,
    IC_SUCCESS = 0x03,
    IC_ERROR = 0x04,
    IC_SIGNAL = 0x05,
    IC_RESPONSE = 0x06,
    IC_UNKNOWN = 0x0f
};

enum InstructionDataCode {
    IDC_NONE = 0x00,
    IDC_SETTINGS = 0x01,
    IDC_STATE = 0x02,
    IDC_ID = 0x03,
    IDC_INTERRUPT_PIN = 0x04,
    IDC_REMOTE_TIMESTAMP = 0x05,
    IDC_STATE_FIX_SETTINGS = 0x06,
#ifdef MEM_32KB
    IDC_SWITCH_COUNTING_SETTINGS = 0x07,
    IDC_CLEAR_SWITCH_COUNT = 0x08,
#endif
    IDC_RELAY_STATE = 0x09,
#ifdef MEM_32KB
    IDC_RELAY_DISABLED_TEMP = 0x0a,
    IDC_RELAY_SWITCHED_ON = 0x0b,
    IDC_RELAY_MONITOR_ON = 0x0c,
    IDC_RELAY_CONTROL_ON = 0x0d,
    IDC_ALL = 0x0e,
#endif
    IDC_VERSION = 0x0f,
    IDC_CURRENT_TIME = 0x10,
    IDC_CONTACT_WAIT_DATA = 0x11,
    IDC_FIX_DATA = 0x12,
    IDC_SWITCH_DATA = 0x13,
    IDC_GET_TIME_STAMP = 0x14,
    IDC_RELAY_STATE_CHANGED = 0x15,
    IDC_MONITORING_STATE_CHANGED = 0x16,
    IDC_CONTROL_STATE_CHANGED = 0x17,
    IDC_GET_CYCLES_STATISTICS = 0x18,
    IDC_STATE_FIX_TRY = 0x19,
    IDC_UNKNOWN = 0xff
};

enum ErrorCode {
    OK = 0x00,
    E_REQUEST_DATA_NO_VALUE = 0x01,
    E_INSTRUCTION_UNRECOGIZED = 0x02,
    E_COMMAND_EMPTY = 0x03,
    E_COMMAND_SIZE_OVERFLOW = 0x04,
    E_INSTRUCTION_WRONG_START = 0x05,
    E_WRITE_MAX_ATTEMPTS_EXCEDED = 0x06,
    E_UNDEFINED_OPERATION = 0x07,
    E_RELAY_COUNT_OVERFLOW = 0x08,
    E_RELAY_COUNT_AND_DATA_MISMATCH = 0x09,
    E_RELAY_INDEX_OUT_OF_RANGE = 0x0a,
    E_SWITCH_COUNT_MAX_VALUE_OVERFLOW = 0x0b,
    E_CONTROL_INTERRUPTED_PIN_NOT_ALLOWED_VALUE = 0x0c,
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

inline void sendStartSignal(InstructionDataCode code) {
    sendSerial(IC_NONE);
    sendSerial(IC_SIGNAL);
    sendSerial(code);
}

#endif //RELAYCONTROLLER_COMMUNICATIONPROTOCOL_H
