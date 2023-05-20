//
// Created by valti on 07.05.2023.
//

#include "utils.h"


void setBit(uint16_t &word, uint8_t bitNo, bool bitValue) {
    if (bitValue) {
        word |= (1 << bitNo);
    } else {
        word &= ~(1 << bitNo);
    }
}

void setBit(uint8_t &word, uint8_t bitNo, bool bitValue) {
    if (bitValue) {
        word |= (1 << bitNo);
    } else {
        word &= ~(1 << bitNo);
    }
}