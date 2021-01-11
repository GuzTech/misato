from nmigen import *
from nmigen.back import verilog
from nmigen.build import Platform

from isa import *

from typing import List


class ALU(Elaboratable):
    def __init__(self, xlen: XLEN):
        # Inputs
        self.in1 = Signal(xlen.value)
        self.in2 = Signal(xlen.value)
        self.funct3 = Signal(Funct3)
        self.funct7 = Signal(Funct7)

        # Output
        self.out = Signal(xlen.value)

    def ports(self) -> List[Signal]:
        return [
            self.in1,
            self.in2,
            self.funct3,
            self.alt_func,
            self.out
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        # Alternate function bit (ADD/SUB, SRL.SRA)
        alt_func = Signal()
        m.d.comb += alt_func.eq(self.funct7[5])

        with m.Switch(self.funct3):
            with m.Case(Funct3.ADD):
                m.d.comb += self.out.eq(Mux(alt_func,
                                            self.in1 - self.in2, self.in1 + self.in2))
            with m.Case(Funct3.OR):
                m.d.comb += self.out.eq(self.in1 | self.in2)
            with m.Case(Funct3.AND):
                m.d.comb += self.out.eq(self.in1 & self.in2)
            with m.Case(Funct3.XOR):
                m.d.comb += self.out.eq(self.in1 ^ self.in2)
            with m.Case(Funct3.SLT):
                m.d.comb += self.out.eq(Mux(self.in1.as_signed()
                                            < self.in2.as_signed(), 0b1, 0b0))
            with m.Case(Funct3.SLTU):
                m.d.comb += self.out.eq(Mux(self.in1 < self.in2, 0b1, 0b0))
            with m.Case(Funct3.SLL):
                m.d.comb += self.out.eq(self.in1 << self.in2[:5])
            with m.Case(Funct3.SR):
                m.d.comb += self.out.eq(Mux(alt_func, self.in1.as_signed()
                                            >> self.in2[:5], self.in1 >> self.in2[:5]))

        return m


if __name__ == "__main__":
    alu = ALU(xlen=XLEN.RV32)

    with open("alu.v", "w") as file:
        file.write(verilog.convert(alu, ports=alu.ports()))
