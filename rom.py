from math import ceil, log2
from amaranth import *
from amaranth.build import Platform
from amaranth_soc.memory import *
from amaranth_soc.wishbone import *


class ROM(Elaboratable):
    def __init__(self, data):
        self.data = Memory(width=32, depth=len(data), init=data)
        self.size = len(data) * 4
        # Initialize Wishbone bus arbiter.
        self.arb = Arbiter(
            addr_width=ceil(log2(self.size + 1)), data_width=32)
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

        m.d.comb += arb.bus.dat_r.eq(Mux(arb.bus.cyc & arb.bus.stb, rp.data, 0))
        # Always assign the bus address to the read port.
        m.d.comb += rp.addr.eq(arb.bus.adr[2:])

        with m.If(arb.bus.ack):
            m.d.sync += arb.bus.ack.eq(0)

        with m.If(arb.bus.cyc & arb.bus.stb):
            m.d.sync += arb.bus.ack.eq(1)

        # Word-aligned reads.
        with m.If((arb.bus.adr & 0b11) == 0b00):
            m.d.comb += arb.bus.dat_r.eq(rp.data)
        # Un-aligned reads.
        with m.Else():
            m.d.comb += arb.bus.dat_r.eq(
                rp.data << ((arb.bus.adr & 0b11) << 3))

        return m
