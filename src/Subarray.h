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
#ifndef SUBARRAY_H
#define SUBARRAY_H

#include <iostream>
#include <memory>
#include <vector>

#include "BankState.h"
#include "Burst.h"
#include "BusPacket.h"
#include "SimulatorObject.h"
#include "SystemConfiguration.h"

namespace DRAMSim
{
class Subarray //x4
{
    typedef struct _DataStruct
    {
        unsigned row;
        BurstType data; //16x16
        std::shared_ptr<struct _DataStruct> next; //points to next datastruct
    } DataStruct;
    //how about use this in subarray level logic?
  public:
    // functions
    Subarray(ostream& simLog);

    void read(BusPacket* busPacket);
    void write(const BusPacket* busPacket);
    BankState currentState;
    int getRow();

  private:
    // private member
    std::vector<std::shared_ptr<DataStruct>> rowEntries;
    ostream& dramsimLog;
    static std::shared_ptr<DataStruct> searchForRow(unsigned row, std::shared_ptr<DataStruct> head); //memory leak/dealloc easily
    unsigned numCols; 
};
} 
#endif