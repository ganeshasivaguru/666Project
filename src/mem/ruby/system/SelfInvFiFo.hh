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

#ifndef __MEM_RUBY_SYSTEM_SELFINVFIFO_HH__
#define __MEM_RUBY_SYSTEM_SELFINVFIFO_HH__


#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "params/SelfInvFiFo.hh"
#include "sim/sim_object.hh"
#include "debug/Ruby.hh"

namespace gem5
{

namespace ruby
{

struct FIFOEntry
{
  Addr address;
  int VerNo;
  bool isSelfInv;
};

#define FIFO_DEPTH 64

class SelfInvFiFo : public SimObject
{
  public:
    typedef SelfInvFiFoParams Params;
    SelfInvFiFo(const Params &p);
    ~SelfInvFiFo();


    // Public Methods
    // check if the FIFO is full
    bool isFull();

    // push a cache entry to the FIFO
    void pushEntry(Addr in_addr /*, int in_VerNo, bool in_isSelfInv*/);

    int getcurrentHead();

    Addr getAddrofCurrentHead();
    
    //Addr lastEntry(); 
    // pop a cache entry
    Addr popEntry();

    // Just a print operation to print the FIFO content
    void printContents();

  private:
    Addr selfInvQueue[FIFO_DEPTH];
    int currentHead;
    int currentTail;
    int currentSize;

};


} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_SYSTEM_SELFINVFIFO_HH___