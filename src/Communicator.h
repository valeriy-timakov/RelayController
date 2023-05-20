//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_COMMUNICATOR_H
#define RELAYCONTROLLER_COMMUNICATOR_H

#include "Arduino.h"
#include "Settings.h"
#include "RelayController.h"

#define CMD_BUFF_SIZE 30


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
    IDC_INTERRUPT_PIN = 10,
    IDC_CONTACT_READY_WAIT_DELAY = 11,
    IDC_SWITCH_COUNT_INTERVAL_SEC = 12,
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

class Communicator {
public:
    Communicator(Settings &settings, RelayController &relayController) : settings(settings), relayController(relayController) {}
    void setup();
    void idle();
private:
    Settings &settings;
    RelayController &relayController;
    uint8_t cmdBuff[CMD_BUFF_SIZE];
    uint8_t cmdBuffSize = 0;
    uint8_t cmdBuffCurrPos = 0;
    bool commandParsed = false;
    bool commandPocessed = false;
    uint32_t lastPacketTime = 0;
    uint8_t lastPacketSize = 0;

    bool readBinaryCommand();
    void processBinaryInstruction();
    ErrorCode processBinaryRead();
    ErrorCode processBinarySet();
    void sendSettings(bool addResultCode = true);
    uint8_t saveSettings();
    ErrorCode sendState(bool addResultCode = true);
    ErrorCode saveState();
    ErrorCode sendId(bool addResultCode = true);
    ErrorCode saveId();
    ErrorCode sendInterruptPin(bool addResultCode = true);
    ErrorCode sendAll();
    ErrorCode saveAll();
    ErrorCode sendRelayState();
    ErrorCode saveRelayState();
    ErrorCode sendRelayDisabledTemp();
    ErrorCode saveRelayDisabledTemp();
    ErrorCode sendRelaySwitchedOn();
    ErrorCode saveRelaySwitchedOn();
    ErrorCode sendRelayMonitorOn();
    ErrorCode saveInterruptPin();
    ErrorCode sendRelayControlOn();
    ErrorCode send(uint8_t(*getter)(Communicator*, uint8_t), InstructionDataCode dataCode = IDC_UNKNOWN);
    ErrorCode save(void(*setter)(Communicator*, uint8_t, uint8_t));
    ErrorCode readUint8FromCmdBuff(uint8_t &result);
    ErrorCode readUint32FromCommandBuffer(uint32_t &result);
    ErrorCode readRelayIndexFromCmdBuff(uint8_t &result);
    ErrorCode readRelayCountFromCmdBuff(uint8_t &count);
    uint8_t readRelayStateBits(uint8_t relayIndex);
    ErrorCode sendContactReadyWaitDelay();
    ErrorCode saveContactReadyWaitDelay();
    ErrorCode readUint16FromCmdBuff(uint16_t &result);
    ErrorCode sendSwitchCountIntervalSec();
    ErrorCode saveSwitchCountIntervalSec();
#ifdef MEM_32KB
    ErrorCode sendMaxSwitchCount();
    ErrorCode saveMaxSwitchCount();
    ErrorCode clearSwitchCount();
#endif
};


#endif //RELAYCONTROLLER_COMMUNICATOR_H
