from math import ceil, log2
from amaranth import *
from amaranth.build import Platform


class GPIO(Elaboratable):
    def __init__(self):
        # Inputs
        self.i_data = Signal(32)
        self.i_w_en = Signal()

        # Outputs
        self.o_data = Signal(32)

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        with m.If(self.i_w_en):
            m.d.sync += self.o_data.eq(self.i_data)

        return m
