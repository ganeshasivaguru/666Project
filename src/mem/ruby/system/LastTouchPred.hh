/*
 * Copyright (c) 2019-2021 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
 * Unordered buffer of messages that can be inserted such
 * that they can be dequeued after a given delta time has expired.
 */

#ifndef __MEM_RUBY_SYSTEM_LastTouchPred_HH__
#define __MEM_RUBY_SYSTEM_LastTouchPred_HH__

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/trace.hh"
#include "debug/RubyQueue.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/network/dummy_port.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "params/LastTouchPred.hh"
#include "sim/sim_object.hh"
#include "mem/ruby/common/MachineID.hh"

namespace gem5
{

namespace ruby
{

class LastTouchPred: public SimObject
{
    public:
        int LTP_id;// = 0;
        void set_LTP_id(int id);
        int get_LTP_id();
        //execution stats
        volatile int num_wrong_invalidations = 0;
        volatile int num_right_invalidations = 0;
        volatile int num_invalidations_predicted = 0; 
        volatile int num_invalidations = 0;

        typedef LastTouchPredParams Params;
        LastTouchPred(const Params &p);
        std::vector<std::vector<int>> current_sig_table;
        std::vector<std::vector<std::vector<int>>> LTP_sig_table;//(100,-1);
        std::vector<std::vector<std::vector<int>>> last_touched_signature;
        std::deque<int> blocks_to_be_self_inv;
        std::vector<std::pair<int, int>> forward_request_table;

        void increment_invalidations();
        int get_LT_match();
        void add_forward_queue(Addr address, int machine_node);
        void remove_forward_queue(Addr address, int machine_node);
        int search_forward(Addr address, int machine_node);

        void update_table_size(int num_blocks);
        int get_sig_table_size();
        int get_LTP_sig_table_size();
        void incrementAccuracy(Addr block_tag, int value);
        void decrementAccuracy(Addr block_tag, int value);
        void weakenAccuracy(Addr block_tag);
        void strengthenAccuracy(Addr block_tag);
        void updateLastTouchedSignatureTable(Addr block_tag, int value);

        void debug_print();
        void append_self_inv(Addr block_tag);
        Addr pop_self_inv();
        bool blocks_to_be_self_inv_empty();

        //sig table add and get
        void add_new_sig_table(Addr block_tag, PacketPtr pkt);
        int check_self_invalidation(Addr block_tag); //caller should be L1
        //int get_sig_table_value(int block_index);

        //LTP sig table add and get
        //caller should be L2 on L2 block invalidation
        void add_new_sig_LTP_table(Addr block_tag);
        //int get_sig_LTP_table_value(int block_index);

        int check_for_LTP_match(int value, Addr block_tag);

 //public:
    //typedef MessageBufferParams Params;
    //LTP(const Params &p);
};


} // namespace ruby
} // namespace gem5

#endif //__MEM_RUBY_SYSTEM_LTP_HH__
