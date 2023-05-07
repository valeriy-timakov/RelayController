//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_COMMUNICATOR_H
#define RELAYCONTROLLER_COMMUNICATOR_H

#include "Arduino.h"
#include "Settings.h"
#include "RelayController.h"

#define CMD_BUFF_SIZE 30

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
    E_UNDEFINED_CODE = 100

};

static const int MAX_COMMAND_READ_TIME = 100;

class Communicator {
public:
    Communicator(Settings &settings, RelayController &relayController) : settings(settings), relayController(relayController) {}
    void setup();
    void idle();
private:
    Settings &settings;
    RelayController &relayController;
    uint8_t cmdBuff[CMD_BUFF_SIZE];
    uint8_t curCmdBuffPos = 0;
    bool commandParsed = false;
    uint32_t lastPacketTime = 0;
    uint8_t lastPacketSize = 0;

    bool readBinaryCommand();
    void processBinaryInstruction();
    ErrorCode processBinaryRead();
    ErrorCode processBinarySet();
    ErrorCode processBinaryExecute();
    void sendSettings();
    ErrorCode saveSettings();
    ErrorCode sendRelayState();
    uint8_t readRelayStateBits(uint8_t relayIndex);

    ErrorCode sendState();

    ErrorCode saveState();

    ErrorCode saveRelayState();
};


#endif //RELAYCONTROLLER_COMMUNICATOR_H
