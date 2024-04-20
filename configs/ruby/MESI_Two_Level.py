# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2009 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from .Ruby import create_topology, create_directories
from .Ruby import send_evicts

#
# Declare caches used by the protocol
#
class L1Cache(RubyCache): pass
class L2Cache(RubyCache): pass

def define_options(parser):
    return

def create_system(options, full_system, system, dma_ports, bootmem,
                  ruby_system, cpus):

    if buildEnv['PROTOCOL'] != 'MESI_Two_Level':
        fatal("This script requires the MESI_Two_Level protocol to be built.")

    cpu_sequencers = []

    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.  Therefore the l1 controller nodes must be
    # listed before the directory nodes and directory nodes before dma nodes, etc.
    #
    l1_cntrl_nodes = []
    l2_cntrl_nodes = []
    dma_cntrl_nodes = []

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #
    l2_bits = int(math.log(options.num_l2caches, 2))
    block_size_bits = int(math.log(options.cacheline_size, 2))
    print(options.num_cpus)
    LTP_array = [0 for x in range(options.num_cpus)]

    LTP_dir = LastTouchPred()
    for i in range(options.num_cpus):
        LTP = LastTouchPred()
        LTP_array[i] = LTP
        
        #
        # First create the Ruby objects associated with this cpu
        #
        l1i_cache = L1Cache(size = options.l1i_size,
                            assoc = options.l1i_assoc,
                            start_index_bit = block_size_bits,
                            is_icache = True)
        l1d_cache = L1Cache(size = options.l1d_size,
                            assoc = options.l1d_assoc,
                            start_index_bit = block_size_bits,
                            is_icache = False)


        clk_domain = cpus[i].clk_domain

        l1_cntrl = L1Cache_Controller(version = i, L1Icache = l1i_cache,
                                      L1Dcache = l1d_cache,
                                      l2_select_num_bits = l2_bits,
                                      send_evictions = send_evicts(options),
                                      ruby_system = ruby_system,
                                      clk_domain = clk_domain,
                                      transitions_per_cycle = options.ports,
                                      enable_prefetch = False)

        cpu_seq = RubySequencer(version = i,
                                dcache = l1d_cache, clk_domain = clk_domain,
                                ruby_system = ruby_system)

        l1_cntrl.LTP = LTP_array[i]
        l1_cntrl.LTP_L2 = LTP_dir
        l1_cntrl.sequencer = cpu_seq

        exec("ruby_system.l1_cntrl%d = l1_cntrl" % i)

        # Add controllers and sequencers to the appropriate lists
        cpu_sequencers.append(cpu_seq)
        l1_cntrl_nodes.append(l1_cntrl)

        # Connect the L1 controllers and the network
        l1_cntrl.mandatoryQueue = MessageBuffer()
        l1_cntrl.requestFromL1Cache = MessageBuffer()
        l1_cntrl.requestFromL1Cache.out_port = ruby_system.network.in_port
        l1_cntrl.responseFromL1Cache = MessageBuffer()
        l1_cntrl.responseFromL1Cache.out_port = ruby_system.network.in_port
        l1_cntrl.unblockFromL1Cache = MessageBuffer()
        l1_cntrl.unblockFromL1Cache.out_port = ruby_system.network.in_port

        l1_cntrl.optionalQueue = MessageBuffer()

        l1_cntrl.requestToL1Cache = MessageBuffer()
        l1_cntrl.requestToL1Cache.in_port = ruby_system.network.out_port
        l1_cntrl.responseToL1Cache = MessageBuffer()
        l1_cntrl.responseToL1Cache.in_port = ruby_system.network.out_port

        l1_cntrl.fromDir_self_inv = MessageBuffer(ordered = True)
        l1_cntrl.fromDir_self_inv.in_port = ruby_system.network.out_port

        l1_cntrl.self_inv_queue_out = MessageBuffer(ordered = True)
        l1_cntrl.self_inv_queue_out.out_port = ruby_system.network.in_port

    l2_index_start = block_size_bits + l2_bits

    for i in range(options.num_l2caches):
        #
        # First create the Ruby objects associated with this cpu
        #
        l2_cache = L2Cache(size = options.l2_size,
                           assoc = options.l2_assoc,
                           start_index_bit = l2_index_start)

        l2_cntrl = L2Cache_Controller(version = i,
                                      L2cache = l2_cache,
                                      transitions_per_cycle = options.ports,
                                      ruby_system = ruby_system)

        exec("ruby_system.l2_cntrl%d = l2_cntrl" % i)
        l2_cntrl_nodes.append(l2_cntrl)

        # Connect the L2 controllers and the network
        l2_cntrl.DirRequestFromL2Cache = MessageBuffer()
        l2_cntrl.DirRequestFromL2Cache.out_port = ruby_system.network.in_port
        l2_cntrl.L1RequestFromL2Cache = MessageBuffer()
        l2_cntrl.L1RequestFromL2Cache.out_port = ruby_system.network.in_port
        l2_cntrl.responseFromL2Cache = MessageBuffer()
        l2_cntrl.responseFromL2Cache.out_port = ruby_system.network.in_port

        l2_cntrl.unblockToL2Cache = MessageBuffer()
        l2_cntrl.unblockToL2Cache.in_port = ruby_system.network.out_port
        l2_cntrl.L1RequestToL2Cache = MessageBuffer()
        l2_cntrl.L1RequestToL2Cache.in_port = ruby_system.network.out_port
        l2_cntrl.responseToL2Cache = MessageBuffer()
        l2_cntrl.responseToL2Cache.in_port = ruby_system.network.out_port

        l2_cntrl.toCache_self_inv_queue = MessageBuffer(ordered = True)
        l2_cntrl.toCache_self_inv_queue.out_port = ruby_system.network.in_port

        l2_cntrl.requestFromCache_self_inv = MessageBuffer(ordered = True)
        l2_cntrl.requestFromCache_self_inv.in_port = ruby_system.network.out_port

        l2_cntrl.LTP = LTP_dir
        l2_cntrl.L1_LTP0 = LTP_array[0]
        l2_cntrl.L1_LTP1 = LTP_array[1]
        l2_cntrl.L1_LTP2 = LTP_array[2]
        l2_cntrl.L1_LTP3 = LTP_array[3]
        l2_cntrl.L1_LTP4 = LTP_array[4]
        l2_cntrl.L1_LTP5 = LTP_array[5]
        l2_cntrl.L1_LTP6 = LTP_array[6]
        l2_cntrl.L1_LTP7 = LTP_array[7]
        l2_cntrl.L1_LTP8 = LTP_array[8]
        l2_cntrl.L1_LTP9 = LTP_array[9]
        l2_cntrl.L1_LTP10 = LTP_array[10]
        l2_cntrl.L1_LTP11 = LTP_array[11]
        l2_cntrl.L1_LTP12 = LTP_array[12]
        l2_cntrl.L1_LTP13 = LTP_array[13]
        l2_cntrl.L1_LTP14 = LTP_array[14]
        l2_cntrl.L1_LTP15 = LTP_array[15]
        l2_cntrl.L1_LTP16 = LTP_array[16]
        l2_cntrl.L1_LTP17 = LTP_array[17]
        l2_cntrl.L1_LTP18 = LTP_array[18]
        l2_cntrl.L1_LTP19 = LTP_array[19]
        l2_cntrl.L1_LTP20 = LTP_array[20]
        l2_cntrl.L1_LTP21 = LTP_array[21]
        l2_cntrl.L1_LTP22 = LTP_array[22]
        l2_cntrl.L1_LTP23 = LTP_array[23]
        l2_cntrl.L1_LTP24 = LTP_array[24]
        l2_cntrl.L1_LTP25 = LTP_array[25]
        l2_cntrl.L1_LTP26 = LTP_array[26]
        l2_cntrl.L1_LTP27 = LTP_array[27]
        l2_cntrl.L1_LTP28 = LTP_array[28]
        l2_cntrl.L1_LTP29 = LTP_array[29]
        l2_cntrl.L1_LTP30 = LTP_array[30]
        l2_cntrl.L1_LTP31 = LTP_array[31]
    # Run each of the ruby memory controllers at a ratio of the frequency of
    # the ruby system
    # clk_divider value is a fix to pass regression.
    ruby_system.memctrl_clk_domain = DerivedClockDomain(
                                          clk_domain = ruby_system.clk_domain,
                                          clk_divider = 3)

    mem_dir_cntrl_nodes, rom_dir_cntrl_node = create_directories(
        options, bootmem, ruby_system, system)
    dir_cntrl_nodes = mem_dir_cntrl_nodes[:]
    if rom_dir_cntrl_node is not None:
        dir_cntrl_nodes.append(rom_dir_cntrl_node)
    for dir_cntrl in dir_cntrl_nodes:
        # Connect the directory controllers and the network
        dir_cntrl.requestToDir = MessageBuffer()
        dir_cntrl.requestToDir.in_port = ruby_system.network.out_port
        dir_cntrl.responseToDir = MessageBuffer()
        dir_cntrl.responseToDir.in_port = ruby_system.network.out_port
        dir_cntrl.responseFromDir = MessageBuffer()
        dir_cntrl.responseFromDir.out_port = ruby_system.network.in_port
        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()


    for i, dma_port in enumerate(dma_ports):
        # Create the Ruby objects associated with the dma controller
        dma_seq = DMASequencer(version = i, ruby_system = ruby_system,
                               in_ports = dma_port)

        dma_cntrl = DMA_Controller(version = i, dma_sequencer = dma_seq,
                                   transitions_per_cycle = options.ports,
                                   ruby_system = ruby_system)

        exec("ruby_system.dma_cntrl%d = dma_cntrl" % i)
        dma_cntrl_nodes.append(dma_cntrl)

        # Connect the dma controller to the network
        dma_cntrl.mandatoryQueue = MessageBuffer()
        dma_cntrl.responseFromDir = MessageBuffer(ordered = True)
        dma_cntrl.responseFromDir.in_port = ruby_system.network.out_port
        dma_cntrl.requestToDir = MessageBuffer()
        dma_cntrl.requestToDir.out_port = ruby_system.network.in_port

    all_cntrls = l1_cntrl_nodes + \
                 l2_cntrl_nodes + \
                 dir_cntrl_nodes + \
                 dma_cntrl_nodes

    # Create the io controller and the sequencer
    if full_system:
        io_seq = DMASequencer(version = len(dma_ports),
                              ruby_system = ruby_system)
        ruby_system._io_port = io_seq
        io_controller = DMA_Controller(version = len(dma_ports),
                                       dma_sequencer = io_seq,
                                       ruby_system = ruby_system)
        ruby_system.io_controller = io_controller

        # Connect the dma controller to the network
        io_controller.mandatoryQueue = MessageBuffer()
        io_controller.responseFromDir = MessageBuffer(ordered = True)
        io_controller.responseFromDir.in_port = ruby_system.network.out_port
        io_controller.requestToDir = MessageBuffer()
        io_controller.requestToDir.out_port = ruby_system.network.in_port

        all_cntrls = all_cntrls + [io_controller]

    ruby_system.network.number_of_virtual_networks = 4
    topology = create_topology(all_cntrls, options)
    return (cpu_sequencers, mem_dir_cntrl_nodes, topology)
