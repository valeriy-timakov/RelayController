//
// Created by valti on 07.05.2023.
//

#ifndef RELAYCONTROLLER_UTILS_H
#define RELAYCONTROLLER_UTILS_H

#include "Arduino.h"

//from https://gist.github.com/Jeff-Russ/c9b471158fa7427280e6707d9b11d7d2
#define BIT(pos) ( 1<<(pos) )
#define SET_LSBITS(len) ( BIT(len)-1 ) // the first len bits are '1' and the rest are '0'
#define BF_MASK(start, len) ( SET_LSBITS(len)<<(start) ) // same but with offset
#define CHECK_BIT(y, pos) ( ( 0u == ( (y)&(BIT(pos)) ) ) ? 0u : 1u )

void setBit(uint16_t &word, uint8_t bitNo, bool bitValue);
void setBit(uint8_t &word, uint8_t bitNo, bool bitValue);

#define MILLIS_PER_SECOND 1000

#endif //RELAYCONTROLLER_UTILS_H
