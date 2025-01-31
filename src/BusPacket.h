/*********************************************************************************
 *  Copyright (c) 2010-2011, Elliott Cooper-Balis
 *                             Paul Rosenfeld
 *                             Bruce Jacob
 *                             University of Maryland
 *                             dramninjas [at] gmail [dot] com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

#ifndef BUSPACKET_H
#define BUSPACKET_H

#include <string>

#include "Burst.h"
#include "SystemConfiguration.h"

namespace DRAMSim
{
enum BusPacketType
{
    READ,       // 0
    WRITE,      // 1
    ACTIVATE,   // 2
    PRECHARGE,  // 3
    REF,        // 4
    DATA,       // 5
    RFCSB,
    SUBSEL
};

class BusPacket
{
    BusPacket();
    ostream& dramsimLog;

  public:
    // Fields
    BusPacketType busPacketType;
    unsigned column; //important problem: buspacket has 256bit wide -->need cell logic
    unsigned row;
    unsigned subarray; //or psub; designate psub area by 
    unsigned bank;
    unsigned rank;
    unsigned chan;
    uint64_t physicalAddress;
    BurstType* data;
    std::string tag;

    // Functions
    BusPacket(BusPacketType packtype, uint64_t physicalAddr, unsigned col, unsigned rw, unsigned r,
              unsigned b, BurstType* dat, ostream& simLog);
    BusPacket(BusPacketType packtype, uint64_t physicalAddr, unsigned col, unsigned rw, unsigned r,
              unsigned b, BurstType* dat, ostream& simLog, std::string tg);

    void print();
    void print(uint64_t currentClockCycle, bool dataStart);
    void printData() const;
};

}  // namespace DRAMSim

#endif
