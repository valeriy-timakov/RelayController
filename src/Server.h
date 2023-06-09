//
// Created by valti on 06.05.2023.
//

#ifndef RELAYCONTROLLER_SERVER_H
#define RELAYCONTROLLER_SERVER_H

#include "Arduino.h"
#include "Settings.h"
#include "RelayController.h"
#include "CommunicationProtocol.h"

#define CMD_BUFF_SIZE 30


class Server {
public:
    Server(Settings &settings) : settings(settings) {
        for (unsigned char & i : cmdBuff) {
            i = 0;
        }
    }
    void setup();
    void idle();
private:
    Settings &settings;
    uint8_t cmdBuff[CMD_BUFF_SIZE];
    uint8_t cmdBuffSize = 0;
    uint8_t cmdBuffCurrPos = 0;
    bool commandParsed = false;
    bool commandPocessed = false;
    uint32_t lastPacketTime = 0;
    uint8_t lastPacketSize = 0;
    static uint16_t minCycleDuration;
    static uint16_t maxCycleDuration;
    static uint64_t cyclesCount;
    static uint64_t lastCycleTime;

    static void updateStatistics();
    bool readBinaryCommand();
    void processBinaryInstruction();
    ErrorCode processBinaryRead(InstructionDataCode dataCode);
    ErrorCode processBinarySet(InstructionDataCode dataCode);
    void sendSettings(bool addResultCode = true);
    uint8_t saveSettings();
    ErrorCode sendState(bool addResultCode = true);
    ErrorCode saveState();
    ErrorCode sendId(bool addResultCode = true);
    ErrorCode saveId();
#ifdef MEM_32KB
    ErrorCode sendInterruptPin(bool addResultCode = true);
    ErrorCode saveInterruptPin();
    ErrorCode sendAll();
    ErrorCode saveAll();
#endif
    ErrorCode sendRelayState();
    ErrorCode saveRelayState();
#ifdef MEM_32KB
    ErrorCode sendRelayDisabledTemp();
    ErrorCode saveRelayDisabledTemp();
    ErrorCode sendRelaySwitchedOn();
    ErrorCode saveRelaySwitchedOn();
    ErrorCode sendRelayMonitorOn();
    ErrorCode sendRelayControlOn();
    ErrorCode send(uint8_t(*getter)(Server*, uint8_t), InstructionDataCode dataCode = IDC_UNKNOWN);
    ErrorCode save(void(*setter)(Server*, uint8_t, uint8_t));
    ErrorCode sendSwitchCountingSettings();
    ErrorCode saveSwitchCountingSettings();
    ErrorCode clearSwitchCount();
#endif
    ErrorCode sendStateFixSettings();
    ErrorCode saveStateFixSettings();
    static ErrorCode sendRemoteTimestamp();
    ErrorCode saveRemoteTimestamp();
    ErrorCode sendFixData();
#ifdef MEM_32KB
    ErrorCode sendContactWaitData();
    static ErrorCode sendSwitchData();
    ErrorCode readRelayIndexFromCmdBuff(uint8_t &result);
#endif
    ErrorCode readUint8FromCmdBuff(uint8_t &result);
    ErrorCode readUint32FromCommandBuffer(uint32_t &result);
    ErrorCode readRelayCountFromCmdBuff(uint8_t &count);
    static uint8_t readRelayStateBits(uint8_t relayIndex);
    ErrorCode readUint16FromCmdBuff(uint16_t &result);
};


#endif //RELAYCONTROLLER_SERVER_H
