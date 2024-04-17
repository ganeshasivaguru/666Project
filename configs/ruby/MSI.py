#Copyright (c) 2017 Jason Power
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

""" This file creates a set of Ruby caches, the Ruby network, and a simple
point-to-point topology.
See Part 3 in the Learning gem5 book:
http://gem5.org/documentation/learning_gem5/part3/MSIintro

IMPORTANT: If you modify this file, it's likely that the Learning gem5 book
           also needs to be updated. For now, email Jason <jason@lowepower.com>

"""

import math

from m5.defines import buildEnv
from m5.objects import *
from .Ruby import create_topology, create_directories
from .Ruby import send_evicts
from m5.util import (
    fatal,
    panic,
)

def define_options(parser):
    return

class L1Cache(RubyCache):
    dataAccessLatency = 1
    tagAccessLatency = 1

def define_options(parser):
    return

def create_system(options, full_system, system, dma_ports, bootmem,
                  ruby_system,cpus):

    if buildEnv['PROTOCOL'] != 'MSI':
        panic("This script requires the MSI protocol to be built.")

    cpu_sequencers = []

    l1_cntrl_nodes = []
    dma_cntrl_nodes = []

    block_size_bits = int(math.log(options.cacheline_size,2))

    for i in range(options.num_cpus):

        cache = L1Cache(size = options.l1d_size,
                        assoc = options.l1d_assoc,
                        start_index_bit = block_size_bits)

        clk_domain = cpus[i].clk_domain

        l1_cntrl = L1Cache_Controller(version=i, cacheMemory=cache,
                                    send_evictions=send_evicts(options),
                                    transitions_per_cycle=options.ports,
                                    clk_domain=clk_domain,
                                    ruby_system=ruby_system)

        cpu_seq = RubySequencer(version=i, dcache=cache,
                                clk_domain=clk_domain,
                                ruby_system=ruby_system)

        l1_cntrl.sequencer = cpu_seq

        exec("ruby_system.l1_cntrl%d = l1_cntrl" % i)

        selfInvFiFo = SelfInvFiFo()
        l1_cntrl.selfInvFIFO = selfInvFiFo

        cpu_sequencers.append(cpu_seq)
        l1_cntrl_nodes.append(l1_cntrl)

        l1_cntrl.mandatoryQueue = MessageBuffer()
        l1_cntrl.requestToDir = MessageBuffer(ordered = True)
        l1_cntrl.requestToDir.out_port = ruby_system.network.in_port
        l1_cntrl.responseToDirOrSibling = MessageBuffer(ordered = True)
        l1_cntrl.responseToDirOrSibling.out_port = ruby_system.network.in_port
        l1_cntrl.forwardFromDir = MessageBuffer(ordered = True)
        l1_cntrl.forwardFromDir.in_port = ruby_system.network.out_port
        l1_cntrl.responseFromDirOrSibling = MessageBuffer(ordered = True)
        l1_cntrl.responseFromDirOrSibling.in_port = \
                                    ruby_system.network.out_port
        l1_cntrl.selfInvQueueIn = MessageBuffer(ordered = True)
        l1_cntrl.selfInvQueueOut = MessageBuffer(ordered = True)
        #l1_cntrl.selfInvQueueIn.in_port = l1_cntrl.selfInvQueueOut.out_port
        l1_cntrl.selfInvQueueIn = l1_cntrl.selfInvQueueOut

        phys_mem_size = sum([r.size() for r in system.mem_ranges])
        assert(phys_mem_size % options.num_dirs == 0)
        mem_module_size = phys_mem_size / options.num_dirs

    ruby_system.memctrl_clk_domain = DerivedClockDomain(
        clk_domain = ruby_system.clk_domain,
        clk_divider=3)

    mem_dir_cntrl_nodes,rom_dir_cntrl_node = create_directories(
            options,bootmem,ruby_system,system
        )
    dir_cntrl_nodes = mem_dir_cntrl_nodes[:]
    if rom_dir_cntrl_node is not None:
        dir_cntrl_nodes.append(rom_dir_cntrl_node)
    for dir_cntrl in dir_cntrl_nodes:
        dir_cntrl.requestFromCache = MessageBuffer(ordered = True)
        dir_cntrl.requestFromCache.in_port = ruby_system.network.out_port
        dir_cntrl.responseFromCache = MessageBuffer(ordered = True)
        dir_cntrl.responseFromCache.in_port = ruby_system.network.out_port

        dir_cntrl.responseToCache = MessageBuffer(ordered=True)
        dir_cntrl.responseToCache.out_port = ruby_system.network.in_port
        dir_cntrl.forwardToCache = MessageBuffer(ordered=True)
        dir_cntrl.forwardToCache.out_port = ruby_system.network.in_port

        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()

    for i, dma_port in enumerate(dma_ports):
        dma_seq = DMASequencer(version=i,
                                   ruby_system = ruby_system)

        dma_cntrl = DMA_Controller(version=i,
                            dma_sequencer = dma_seq,
                            transitions_per_cycle = options.ports,
                            ruby_system = ruby_system)

        exec("ruby_system.dma_cntrl%d = dma_cntrl" % i)
        exec("ruby_system.dma_cntrl%d.dma_sequencer.in_ports = dma_port" % i)
        dma_cntrl_nodes.append(dma_cntrl)

        dma_cntrl.mandatoryQueue = MessageBuffer()
        dma_cntrl.requestToDir = MessageBuffer()
        dma_cntrl.requestToDir.out_port = ruby_system.network.in_port
        dma_cntrl.responseFromDir = MessageBuffer(ordered = True)
        dma_cntrl.responseFromDir.in_port = ruby_system.network.out_port

    all_cntrls = l1_cntrl_nodes + dir_cntrl_nodes + dma_cntrl_nodes

    if full_system:
        io_seq = DMASequener(version=len(dma_ports),ruby_system=ruby_system)
        ruby_system.io_port = io_seq
        io_controller = DMA_Controller(version=len(dma_ports),
                                dma_sequencer = io_seq,
                                ruby_system = ruby_system)

        ruby_system.io_controller = io_controller
        io_controller.mandatoryQueue = MessageBuffer()
        io_controller.requestToDir = MessageBuffer()
        io_controller.requestToDir.out_port = ruby_system.network.in_port
        io_controller.responseFromDir = MessageBuffer(ordered=True)
        io_controller.responseFromDir.in_port = ruby_system.network.out_port

        all_cntrls = all_cntrls + [io_controller]

    ruby_system.network.number_of_virtual_networks = 3
    topology = create_topology(all_cntrls,options)
    return(cpu_sequencers,mem_dir_cntrl_nodes,topology)

