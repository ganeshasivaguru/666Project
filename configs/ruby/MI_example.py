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

def define_options(parser):
    return

def create_system(options, full_system, system, dma_ports, bootmem,
                  ruby_system, cpus):

    if buildEnv['PROTOCOL'] != 'MI_example':
        panic("This script requires the MI_example protocol to be built.")

    cpu_sequencers = []

    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.  Therefore the l1 controller nodes must be
    # listed before the directory nodes and directory nodes before dma nodes, etc.
    #
    l1_cntrl_nodes = []
    dma_cntrl_nodes = []
    # Set up the system
    system.mem_mode = 'timing'               # Use timing accesses
    #system.mem_ranges = [AddrRange('512MB')] # Create an address range
    # Create a simple memory controller and connect it to the membus
    #system.mem_ctrl = SimpleMemory(latency="50ns", bandwidth="0GB/s")
    #system.mem_ctrl.range = system.mem_ranges[0]
    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #
    block_size_bits = int(math.log(options.cacheline_size, 2))
    LTP_array = [0 for x in range(options.num_cpus)]
    print(options.num_cpus)
    for i in range(options.num_cpus):
        LTP = LastTouchPred()
        LTP_array[i] = LTP
        
        #
        # First create the Ruby objects associated with this cpu
        # Only one cache exists for this protocol, so by default use the L1D
        # config parameters.
        print("L1 size = " + str(options.l1d_size))
        cache = L1Cache(size = options.l1d_size,
                        assoc = options.l1d_assoc,
                        start_index_bit = block_size_bits)


        clk_domain = cpus[i].clk_domain

        # Only one unified L1 cache exists. Can cache instructions and data.
        l1_cntrl = L1Cache_Controller(version=i, cacheMemory=cache,
                                      send_evictions=send_evicts(options),
                                      transitions_per_cycle=options.ports,
                                      clk_domain=clk_domain,
                                      ruby_system=ruby_system)

        cpu_seq = RubySequencer(version=i, dcache=cache,
                                clk_domain=clk_domain, ruby_system=ruby_system)

        l1_cntrl.sequencer = cpu_seq
        exec("ruby_system.l1_cntrl%d = l1_cntrl" % i)

        # Add controllers and sequencers to the appropriate lists
        cpu_sequencers.append(cpu_seq)
        l1_cntrl_nodes.append(l1_cntrl)
        l1_cntrl.LTP = LTP_array[i]
        #l1_cntrl.LTP.set_LTP_id(i)
        # Connect the L1 controllers and the network

        #l1_cntrl.mandatoryQueue = MessageBuffer()
        #l1_cntrl.requestFromCache = MessageBuffer(ordered = True)
        #l1_cntrl.requestFromCache.out_port = ruby_system.network.in_port
        #l1_cntrl.responseFromCache = MessageBuffer(ordered = True)
        # l1_cntrl.responseFromCache.out_port = ruby_system.network.in_port
        # l1_cntrl.forwardToCache = MessageBuffer(ordered = True)
        # l1_cntrl.forwardToCache.in_port = ruby_system.network.out_port
        # l1_cntrl.responseToCache = MessageBuffer(ordered = True)
        # l1_cntrl.responseToCache.in_port = ruby_system.network.out_port
        #l1_cntrl.requestFromCache = MessageBuffer(ordered = True)
        #l1_cntrl.requestFromCache.in_port = ruby_system.network.out_port
        #l1_cntrl.responseFromCache = MessageBuffer(ordered = True)
        #l1_cntrl.responseFromCache.in_port = ruby_system.network.out_port

        #self.responseToCache = MessageBuffer(ordered = True)
        #self.responseToCache.out_port = ruby_system.network.in_port
        #self.forwardToCache = MessageBuffer(ordered = True)
        #self.forwardToCache.out_port = ruby_system.network.in_port
        # All message buffers must be created and connected to the
        # general Ruby network. In this case, "in_port/out_port" don't
        # mean the same thing as normal gem5 ports. If a MessageBuffer
        # is a "to" buffer (i.e., out) then you use the "out_port",
        # otherwise, the in_port.
        l1_cntrl.mandatoryQueue = MessageBuffer()
        #l1_cntrl.self_inv_queue_out = MessageBuffer()
        #l1_cntrl.self_inv_queue_in = MessageBuffer()
        #l1_cntrl.L1Cache_out_in = MessageBuffer()
        #l1_cntrl.L1Cache_out_in.in_port = L1Cache_out_in.out_port

        l1_cntrl.requestToDir = MessageBuffer(ordered = True)
        l1_cntrl.requestToDir.out_port = ruby_system.network.in_port
        l1_cntrl.responseToDirOrSibling = MessageBuffer(ordered = True)
        l1_cntrl.responseToDirOrSibling.out_port = ruby_system.network.in_port
        l1_cntrl.forwardFromDir = MessageBuffer(ordered = True)
        l1_cntrl.forwardFromDir.in_port = ruby_system.network.out_port
        l1_cntrl.responseFromDirOrSibling = MessageBuffer(ordered = True)
        l1_cntrl.responseFromDirOrSibling.in_port=ruby_system.network.out_port

        l1_cntrl.fromDir_self_inv = MessageBuffer(ordered = True)
        l1_cntrl.fromDir_self_inv.in_port = ruby_system.network.out_port

        #l1_cntrl.self_inv_queue_in = MessageBuffer(ordered = True)
        l1_cntrl.self_inv_queue_out = MessageBuffer(ordered = True)
        #l1_cntrl.self_inv_queue_in.in_port = ruby_system.network.out_port#self_inv_queue_out.out_port#
        l1_cntrl.self_inv_queue_out.out_port = ruby_system.network.in_port

    phys_mem_size = sum([r.size() for r in system.mem_ranges])
    assert(phys_mem_size % options.num_dirs == 0)
    mem_module_size = phys_mem_size / options.num_dirs

    # Run each of the ruby memory controllers at a ratio of the frequency of
    # the ruby system.
    # clk_divider value is a fix to pass regression.
    ruby_system.memctrl_clk_domain = DerivedClockDomain(
                                          clk_domain=ruby_system.clk_domain,
                                          clk_divider=3)

    mem_dir_cntrl_nodes, rom_dir_cntrl_node = create_directories(
        options, bootmem, ruby_system, system)
    dir_cntrl_nodes = mem_dir_cntrl_nodes[:]
    if rom_dir_cntrl_node is not None:
        dir_cntrl_nodes.append(rom_dir_cntrl_node)
    for dir_cntrl in dir_cntrl_nodes:
        # Connect the directory controllers and the network
        #dir_cntrl.requestToDir = MessageBuffer(ordered = True)
        #dir_cntrl.requestToDir.in_port = ruby_system.network.out_port
        #dir_cntrl.dmaRequestToDir = MessageBuffer(ordered = True)
        #dir_cntrl.dmaRequestToDir.in_port = ruby_system.network.out_port

        #dir_cntrl.responseFromDir = MessageBuffer()
        #dir_cntrl.responseFromDir.out_port = ruby_system.network.in_port
        #dir_cntrl.dmaResponseFromDir = MessageBuffer(ordered = True)
        #dir_cntrl.dmaResponseFromDir.out_port = ruby_system.network.in_port
        #dir_cntrl.forwardFromDir = MessageBuffer()
        #dir_cntrl.forwardFromDir.out_port = ruby_system.network.in_port
        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()
        dir_cntrl.toCache_self_inv_queue = MessageBuffer(ordered = True)

        #NIcole added below
        dir_cntrl.forwardToCache = MessageBuffer(ordered = True)
        dir_cntrl.forwardToCache.out_port = ruby_system.network.in_port
        dir_cntrl.requestFromCache = MessageBuffer(ordered = True)
        dir_cntrl.requestFromCache.in_port = ruby_system.network.out_port
        dir_cntrl.responseFromCache = MessageBuffer(ordered = True)
        dir_cntrl.responseFromCache.in_port = ruby_system.network.out_port
        dir_cntrl.responseToCache = MessageBuffer(ordered = True)
        dir_cntrl.responseToCache.out_port = ruby_system.network.in_port

        dir_cntrl.requestFromCache_self_inv = MessageBuffer(ordered = True)
        dir_cntrl.requestFromCache_self_inv.in_port = ruby_system.network.out_port

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

    ruby_system.network.number_of_virtual_networks = 3
    topology = create_topology(all_cntrls, options)
    return (cpu_sequencers, mem_dir_cntrl_nodes, topology)
