from math import ceil, log2
from typing import List


from amaranth import *
from amaranth.build import Platform
from amaranth.cli import main_parser, main_runner
from amaranth_soc.memory import *
from amaranth_soc.wishbone import *


class ROM(Elaboratable):
    def __init__(self, data: List):
        self.data = Memory(width=32, depth=len(data), init=data)
        self.size = len(data) * 4
        # Initialize Wishbone bus arbiter.
        self.arb = Arbiter(
            addr_width=ceil(log2(self.size + 1)),
            data_width=32,
            features=["stall", "err"])
        self.arb.bus.memory_map = MemoryMap(
            addr_width=self.arb.bus.addr_width,
            data_width=self.arb.bus.data_width,
            alignment=0)

    def new_bus(self) -> Interface:
        # Initialize a new Wishbone bus interface.
        bus = Interface(addr_width=self.arb.bus.addr_width,
                        data_width=self.arb.bus.data_width,
                        features=["stall", "err"])
        bus.memory_map = MemoryMap(addr_width=bus.addr_width,
                                   data_width=bus.data_width,
                                   alignment=0)
        self.arb.add(bus)

        return bus

    def ports(self) -> List[Signal]:
        return [
            self.arb.bus.stb,
            self.arb.bus.cyc,
            self.arb.bus.adr,
            self.arb.bus.sel,
            self.arb.bus.dat_r,
            self.arb.bus.ack,
            self.arb.bus.stall,
            self.arb.bus.err,
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.submodules.arb = arb = self.arb
        m.submodules.rp = rp = self.data.read_port()

        m.d.comb += arb.bus.dat_r.eq(Mux(arb.bus.cyc & arb.bus.stb, rp.data, 0))
        # Always assign the bus address to the read port.
        m.d.comb += rp.addr.eq(arb.bus.adr[2:])

        # ACK must be zero after a reset
        with m.If(ResetSignal()):
            m.d.sync += arb.bus.ack.eq(0)
        with m.Else():
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


if __name__ == "__main__":
    formal = True

    data = [0xFFFF_FFFF for n in range(32)]

    top = Module()
    top.submodules.rom = rom = ROM(data=data)
    bus = rom.new_bus()

    if formal:
        from f_wb_master import F_WB_Master
        top.submodules.wbm = wbm = F_WB_Master(addr_width=32,
                                               data_width=32)

        top.d.comb += bus.cyc.eq(wbm.cyc)
        top.d.comb += bus.stb.eq(wbm.stb)
        top.d.comb += bus.adr.eq(wbm.adr)
        top.d.comb += bus.sel.eq(wbm.sel)
        top.d.comb += wbm.dat_w.eq(bus.dat_r)
        top.d.comb += wbm.ack.eq(bus.ack)
        top.d.comb += wbm.stall.eq(bus.stall)
        top.d.comb += wbm.err.eq(bus.err)

    parser = main_parser()
    args = parser.parse_args()
    main_runner(parser, args, top)