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

#include "mem/ruby/system/LastTouchPred.hh"

#include <cassert>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/stl_helpers.hh"
#include "debug/RubyQueue.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

//using stl_helpers::operator<<;

LastTouchPred::LastTouchPred(const Params &p)
    :SimObject(p)
{
    //current_sig_table.resize(100, -1);
    //LTP_sig_table.resize(100, std::vector<int>(5));
    //current_sig_table(100,-1);
    //LTP_sig_table(100,-1);
};

void
LastTouchPred::update_table_size(int num_blocks){
    //current_sig_table.resize(num_blocks, -1);
    //LTP_sig_table.resize(num_blocks, std::vector<int>(0));

    return;
}
void
LastTouchPred::debug_print(){
    inform("debug print\n");
}

void
LastTouchPred::set_LTP_id(MachineID id){
    LTP_id = id;
}

MachineID
LastTouchPred::get_LTP_id(){
    return LTP_id;
}

void
LastTouchPred::increment_invalidations(){
    num_invalidations++;
    //inform("id = %d, num invalidations = %d\n",LTP_id,num_invalidations );
}

int
LastTouchPred::get_sig_table_size(){
    return current_sig_table.size();
}

int
LastTouchPred::get_LTP_sig_table_size(){
    return LTP_sig_table.size();
}

void 
LastTouchPred::append_self_inv(Addr block_tag){
    blocks_to_be_self_inv.push_back(block_tag);
}

Addr 
LastTouchPred::pop_self_inv(){
    Addr front = blocks_to_be_self_inv.front();
    blocks_to_be_self_inv.pop_front();
    return front;
}

bool
LastTouchPred::blocks_to_be_self_inv_empty(){
    if(blocks_to_be_self_inv.empty())
        return true;
    else
        return false;
}

void
LastTouchPred::add_forward_queue(Addr address, MachineID machine_node){
    
    forward_request_table.push_back(std::make_pair(address, machine_node));
}

void
LastTouchPred::remove_forward_queue(Addr address, MachineID machine_node){
    int found_entry = 0;
    for(int i = 0; i < forward_request_table.size(); i++ ){
        if(forward_request_table[i].first == address && forward_request_table[i].second == machine_node){
            forward_request_table.erase(forward_request_table.begin() + i);
            found_entry = 1;
        }
    }
    assert(found_entry);
}

int
LastTouchPred::search_forward(Addr address, MachineID machine_node){
     int found_entry = 0;
    for(int i = 0; i < forward_request_table.size(); i++ ){
        if(forward_request_table[i].first == address && forward_request_table[i].second == machine_node){
            //forward_request_table.erase(i);
            found_entry = 1;
        }
    }
    return found_entry;
}


void
LastTouchPred::incrementAccuracy(Addr block_tag, int value){
    num_right_invalidations++;
    //inform("id = %d, num right = %d\n",LTP_id,num_right_invalidations);
    //inform("increment\n");
    for (int i = 0; i <LTP_sig_table.size(); i++){
        for(int x = 0; x <LTP_sig_table[i].size(); x++){
            if(LTP_sig_table[i][x][0] == block_tag){
                if(LTP_sig_table[i][x][1] == value){
                    if(LTP_sig_table[i][x][2] != 3){
                        LTP_sig_table[i][x][2]++;
                    }
                }
                
            }
        }
    }
}
void
LastTouchPred::decrementAccuracy(Addr block_tag, int value){
    num_wrong_invalidations++;
    //inform("id = %d, num wrong = %d\n",LTP_id,num_wrong_invalidations);
    //inform("decrement\n");
    for (int i = 0; i <LTP_sig_table.size(); i++){
        for(int x = 0; x <LTP_sig_table[i].size(); x++){
            if(LTP_sig_table[i][x][0] == block_tag){
                if(LTP_sig_table[i][x][1] == value){
                    if(LTP_sig_table[i][x][2] > 0){
                        LTP_sig_table[i][x][2]--;
                    }
                }
                
            }
        }
    }
}
void 
LastTouchPred::weakenAccuracy(Addr block_tag){
    //search through last touch array and find last LT signature for this block
    for(int i = 0; i < last_touched_signature.size(); i++){
        if(last_touched_signature[i].size() > 0 && last_touched_signature[i][0][0] == block_tag ){
            decrementAccuracy(block_tag,last_touched_signature[i][0][1]);
            last_touched_signature[i].erase(last_touched_signature[i].begin());
        }
    }
    
}

void 
LastTouchPred::strengthenAccuracy(Addr block_tag){
    //search through last touch array and find last LT signature for this block
    for(int i = 0; i < last_touched_signature.size(); i++){
        if(last_touched_signature[i].size() > 0 && last_touched_signature[i][0][0] == block_tag ){
            incrementAccuracy(block_tag,last_touched_signature[i][0][1]);
            last_touched_signature[i].erase(last_touched_signature[i].begin());
        }
    }
    
}
void
LastTouchPred::add_new_sig_table(Addr block_tag,Packet* pkt){

    //inform("\n Add new trace\n");
    //return value is whether to self invalidate or not
    int pc = pkt->req->getPC();
    int value = pc & 0x1FFF;//13 bit truncated addition encoding
    int LT_match = 0;
    int self_invalidate = 0;
    int found_block = 0;
    std::vector<int> new_vect{block_tag, value,2}; //  signature encoding
    for (int i = 0; i <current_sig_table.size(); i++){
        if (current_sig_table[i][0] == block_tag){
            //inform("\nfound tag = %x\n", block_tag);
            found_block = 1;
            //block already has running trace so add the value
            // but first check if the new value is the LT
            //LT_match = check_for_LTP_match(value, block_tag);
            /*if (LT_match){

                self_invalidate = 1;
            }*/
            //else
                current_sig_table[i][1] = value + current_sig_table[i][1];
        }
    }
    if (!found_block)
        current_sig_table.push_back(new_vect);
    return;// self_invalidate;

}

int
LastTouchPred::check_self_invalidation(Addr block_tag){
    //int test = current_sig_table.size()
    //inform("\n check for self invalidation \n");
    //get current trace value
    int found_block = 0;
    //int self_invalidate = 0
    int LT_match = 0;
    for (int i = 0; i <current_sig_table.size(); i++){
        //inform("iterating, block_tag = %x\n",block_tag);
        if (current_sig_table[i][0] == block_tag){
            //compare
            //inform("in array, block_tag = %x\n",block_tag);
            LT_match = check_for_LTP_match(current_sig_table[i][1], block_tag);
            if (LT_match){
                current_sig_table[i][1] = 0;
            }
        }

    }
    /*if (block_tag == 0x400){
       LT_match = 1;
    }*/

    //inform("match = %d\n",LT_match);
    if (LT_match){
        num_invalidations_predicted++;
        //inform("id = %d, num predicted = %d\n",LTP_id,num_invalidations_predicted);
        inform("address %x matched for LT and to be self invalidated\n",block_tag);
    }

    return LT_match;

}

void
LastTouchPred::add_new_sig_LTP_table(Addr block_tag){
    //inform("\n NEW LTP for block = %x\n",block_tag);
    int value = 0;
    int curr_found_block = 0;
    //search for trace signature
    for (int i = 0; i <current_sig_table.size(); i++){
        if (current_sig_table[i][0] == block_tag){
            //capture the current trace and make it into LT value
            value = current_sig_table[i][1];
            //reset trace addition to 0
            current_sig_table[i][1] = 0;
            curr_found_block = 1;
        }
    }
    //inform("searched current\n");
    if (curr_found_block){
        //iterates through table to see if block tag is present
        if (value != 0){
            //inform("value!=0\n");
            std::vector<int> new_vect{block_tag, value,2};
            int found_block = 0;
            //inform("before size\n");
            //inform("size = %d\n",LTP_sig_table.size());

            for (int x = 0; x <LTP_sig_table.size(); x++){
                //inform("in for\n");
                if (LTP_sig_table[x][0][0] == block_tag){
                    // already a LT trace so add new value
                    LTP_sig_table[x].push_back(new_vect);
                    found_block = 1;
                }
            }
            if (found_block == 0){
                //inform("did not find\n");
                //did not find block tag in table so add to table
                std::vector<std::vector<int>> add_vect;
                add_vect.push_back(new_vect);
                LTP_sig_table.push_back(add_vect);
            }
        }
    }

    //inform("done\n");
    return;

}
/*
int
LastTouchPred::get_sig_table_value(int block_index){

    return current_sig_table[block_index];
}*/
/*
int
LastTouchPred::get_sig_LTP_table_value(int block_index){

    return LTP_sig_table[block_index][0];
}*/
void
LastTouchPred::updateLastTouchedSignatureTable(Addr block_tag, int value){
    //iterate through the table and look for a match for the block
    int foundBlock = 0;
    std::vector new_vect = {block_tag, value};
    for (int i = 0; i < last_touched_signature.size(); i++){
        if(last_touched_signature[i].size() == 0  || last_touched_signature[i][0][0] == block_tag ){
            //update last touched

            last_touched_signature[i].push_back(new_vect);
            foundBlock = 1;
        }
    }
    if(!foundBlock){
        //add block
        std::vector<int> new_vect = {block_tag,value};
        std::vector<std::vector<int>> add_vect;

        add_vect.push_back(new_vect);
        last_touched_signature.push_back(add_vect);
    }
}

int
LastTouchPred::check_for_LTP_match(int value, Addr block_tag){
    int LT_match = 0;
    //called on a new ld, st added to table
    //on add, if there is a LTP match, 1 is
    // returned and the caller knows to self-invalidate
    for (int i = 0; i < LTP_sig_table.size(); i++){
        if (LTP_sig_table[i].size() != 0){
            //there is one LT for this block
            //check block tag in index 0
            if (LTP_sig_table[i][0][0] == block_tag ){
                //this index is for the target block, so check signature/value
                if (LTP_sig_table[i][0][1] == value && LTP_sig_table[i][0][1] >= 2){
                    LT_match = 1;
                }
            }
        }
    }
    //if(block_tag == 0x400){
        //LT_match = 1;
    //}
    if(LT_match){
        updateLastTouchedSignatureTable(block_tag,value);
    }
    

    return LT_match;
}


} // namespace ruby
} // namespace gem5
