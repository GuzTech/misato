from amaranth import *
from amaranth.back import verilog
from amaranth.build import Platform

from isa import *

from typing import List
from enum import Enum, unique


@unique
class BInsn(Enum):
    BEQ  = 0b000
    BNE  = 0b001
    BLT  = 0b100
    BGE  = 0b101
    BLTU = 0b110
    BGEU = 0b111


class Branch(Elaboratable):
    def __init__(self, xlen: XLEN):
        # Inputs
        self.in1 = Signal(xlen.value)
        self.in2 = Signal(xlen.value)
        self.br_insn = Signal(BInsn)

        # Output
        self.take_branch = Signal()

    def ports(self) -> List[Signal]:
        return [
            self.in1,
            self.in2,
            self.br_insn,
            self.take_branch
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        with m.Switch(self.br_insn):
            with m.Case(BInsn.BEQ):
                m.d.comb += self.take_branch.eq(self.in1 == self.in2)
            with m.Case(BInsn.BNE):
                m.d.comb += self.take_branch.eq(self.in1 != self.in2)
            with m.Case(BInsn.BLT):
                m.d.comb += self.take_branch.eq(self.in1.as_signed() < self.in2.as_signed())
            with m.Case(BInsn.BGE):
                m.d.comb += self.take_branch.eq(self.in1.as_signed() >= self.in2.as_signed())
            with m.Case(BInsn.BLTU):
                m.d.comb += self.take_branch.eq(self.in1 < self.in2)
            with m.Case(BInsn.BGEU):
                m.d.comb += self.take_branch.eq(self.in1 >= self.in2)
            with m.Default():
                m.d.comb += self.take_branch.eq(0)

        return m


if __name__ == "__main__":
    br = Branch(xlen=XLEN.RV32)

    with open("branch.v", "w") as file:
        file.write(verilog.convert(br, ports=br.ports()))
