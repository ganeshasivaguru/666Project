/*
 * Copyright (c) 2020-2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/ruby/system/SelfInvFiFo.hh"
#include "debug/DSI.hh"

namespace gem5
{

namespace ruby
{


SelfInvFiFo::SelfInvFiFo(const Params &p)
  : SimObject(p)
{
  // Just call init in the constructor
  for (int i=0; i < FIFO_DEPTH; i++) {
          selfInvQueue[i] = 0;
  }
  currentHead = 0;
  currentTail = 0;
  currentSize = 0;
}

SelfInvFiFo::~SelfInvFiFo() {


}


bool
SelfInvFiFo::isFull() {

  if (currentSize == FIFO_DEPTH) {
    return true;
  } else {
    return false;
  }

}

//Addr 
//SelfInvFiFo::lastEntry() {
//  if(currentTail == 0) {
//    return selfInvQueue[FIFO_DEPTH-1];
//  } else { 
//    return selfInvQueue[currentTail-1];
//  }
//}

void
SelfInvFiFo::pushEntry(Addr in_addr /*, int in_VerNo, bool in_isSelfInv*/) {
  DPRINTF(RubySlicc, "Currenttail is %d\n", currentTail);
  selfInvQueue[currentTail]  = in_addr;
  //selfInvQueue[currentTail].VerNo     = in_VerNo;
  //selfInvQueue[currentTail].isSelfInv = in_isSelfInv;
  currentTail = (currentTail+1)%FIFO_DEPTH; // 64 entries so mod with 64
  currentSize++;
  DPRINTF(RubySlicc, "Pushed the entry\n");
  DPRINTF(RubySlicc, "Current size of FIFO is : %d\n", currentSize);
  DPRINTF(RubySlicc, "Tail after push is %d\n", currentTail);
}

Addr
SelfInvFiFo::popEntry() {

  int prevHead = currentHead;
  currentHead = (currentHead + 1)%FIFO_DEPTH;
  currentSize--;
  return selfInvQueue[prevHead];
}

int SelfInvFiFo::getcurrentHead() {
  return currentHead;
}

Addr SelfInvFiFo::getAddrofCurrentHead(){
  return selfInvQueue[currentHead];
}

void SelfInvFiFo::printContents() {
  for (int i=0; i<FIFO_DEPTH; i++) {
    DPRINTF(DSI, "fifo[%d]:, -> %x\n", i, selfInvQueue[i]);
  }
}

} // namespace ruby
} // namespace gem5