from math import ceil, log2
from amaranth import *
from amaranth.build import Platform
from amaranth_soc.memory import *
from amaranth_soc.wishbone import *


class RAM(Elaboratable):
    def __init__(self, num_words):
        self.data = Memory(width=32, depth=num_words)
        self.size = num_words * 4
        # Initialize Wishbone bus arbiter.
        self.arb = Arbiter(
            addr_width=ceil(log2(self.size + 1)),
            data_width=32)
        self.arb.bus.memory_map = MemoryMap(
            addr_width=self.arb.bus.addr_width,
            data_width=self.arb.bus.data_width,
            alignment=0)

    def new_bus(self) -> Interface:
        # Initialize a new Wishbone bus interface.
        bus = Interface(addr_width=self.arb.bus.addr_width,
                        data_width=self.arb.bus.data_width)
        bus.memory_map = MemoryMap(addr_width=bus.addr_width,
                                   data_width=bus.data_width,
                                   alignment=0)
        self.arb.add(bus)

        return bus

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.submodules.arb = arb = self.arb
        m.submodules.rp = rp = self.data.read_port()
        m.submodules.wp = wp = self.data.write_port()

        m.d.comb += arb.bus.dat_r.eq(Mux(arb.bus.cyc & arb.bus.stb, rp.data, 0))
        # Always assign the bus address to the read and write ports.
        m.d.comb += rp.addr.eq(arb.bus.adr[2:])
        m.d.comb += wp.addr.eq(arb.bus.adr[2:])

        with m.If(arb.bus.ack):
            m.d.sync += arb.bus.ack.eq(0)

        with m.If(arb.bus.cyc & arb.bus.stb):
            m.d.sync += arb.bus.ack.eq(1)

        m.d.comb += arb.bus.dat_w.eq(rp.data)
        m.d.comb += wp.en.eq(0)

        with m.If(self.wbus.cyc & self.wbus.stb):
            with m.If(self.wbus.we):
                m.d.comb += wp.addr.eq(self.wbus.adr[2:])
                m.d.comb += wp.data.eq(self.wbus.dat_w)
                m.d.comb += wp.en.eq(1)
            with m.Else():
                m.d.sync += self.wbus.ack.eq(1)
                m.d.comb += rp.addr.eq(self.wbus.adr[2:])

        return m
