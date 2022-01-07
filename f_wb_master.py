from amaranth import *
from amaranth.cli import main_parser, main_runner
from amaranth.build import Platform
from amaranth.asserts import *
from amaranth_soc.wishbone import *


class F_WB_Master(Elaboratable, Interface):
    def __init__(self,
                 addr_width: int,
                 data_width: int):
        assert(addr_width > 0)
        assert(data_width > 0)

        # Configuration
        self.addr_width = addr_width
        self.data_width = data_width

        Interface.__init__(self,
                           addr_width=addr_width,
                           data_width=data_width,
                           features=["stall", "err"])

    def elaborate(self, platform: Platform) -> Module():
        m = Module()

        with m.If(Past(ResetSignal())):
            m.d.comb += Assert(self.cyc == 0b0)
            m.d.comb += Assert(self.stb == 0b0)

            m.d.comb += Assume(self.ack == 0b0)
            m.d.comb += Assume(self.err == 0b0)

        with m.If(Past(self.err) & Past(self.cyc)):
            m.d.comb += Assert(self.cyc == 0b1)

        return m
