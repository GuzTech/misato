from math import ceil, log2
from nmigen import *
from nmigen.build import Platform
from nmigen_soc.memory import *
from nmigen_soc.wishbone import *


class ROM(Elaboratable):
    def __init__(self, data):
        self.data = data
        self.size = len(self.data) * 4
        # self.arb  = Arbiter(
        #     addr_width=ceil(log2(self.size + 1)), data_width=32)
        self.wbus = Interface(addr_width=ceil(log2(self.size + 1)), data_width=32)
        # self.wbus = Interface(addr_width=14, data_width=32)

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        mem = Memory(width=32, depth=len(self.data), init=self.data)
        # mem = Memory(width=32, depth=4096, init=self.data)
        m.submodules.rp = rp = mem.read_port()

        # size = len(self.data) * 4

        # m.submodules.arb = arb = self.arb
        # arb.bus.memory_map = MemoryMap(
            # addr_width=arb.bus.addr_width, data_width=arb.bus.data_width)
        self.wbus.memory_map = MemoryMap(
            addr_width=self.wbus.addr_width, data_width=self.wbus.data_width)

        # arb.add(self.wbus)

        # m.d.comb += arb.bus.dat_r.eq(rp.data)

        # with m.If(arb.bus.ack):
        #     m.d.sync += arb.bus.ack.eq(0)

        # with m.If(arb.bus.cyc & arb.bus.stb):
        #     m.d.sync += arb.bus.ack.eq(1)
        #     m.d.comb += rp.addr.eq(arb.bus.adr)

        m.d.comb += self.wbus.dat_r.eq(rp.data)

        with m.If(self.wbus.ack):
            m.d.sync += self.wbus.ack.eq(0)

        with m.If(self.wbus.cyc & self.wbus.stb):
            m.d.sync += self.wbus.ack.eq(1)
            m.d.comb += rp.addr.eq(self.wbus.adr[2:])

        return m
