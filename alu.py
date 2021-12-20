from amaranth import *
from amaranth.back import verilog
from amaranth.build import Platform

from isa import *

from typing import List


class ALU(Elaboratable):
    def __init__(self, xlen: XLEN):
        # Inputs
        self.i_in1    = Signal(xlen.value)  # ALU input #1
        self.i_in2    = Signal(xlen.value)  # ALU input #2
        self.i_funct3 = Signal(Funct3)      # Funct3 field
        self.i_funct7 = Signal(Funct7)      # Funct7 field

        # Output
        self.o_out    = Signal(xlen.value)  # ALU output
        self.o_zero   = Signal()            # Result is zero

    def ports(self) -> List[Signal]:
        return [
            self.i_in1,
            self.i_in2,
            self.i_funct3,
            self.alt_func,
            self.o_out,
            self.o_zero
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        # Alternate function bit (ADD/SUB, SRL.SRA)
        alt_func = Signal()
        m.d.comb += alt_func.eq(self.i_funct7[5])

        with m.Switch(self.i_funct3):
            with m.Case(Funct3.ADD):
                m.d.comb += self.o_out.eq(Mux(alt_func,
                                            self.i_in1 - self.i_in2, self.i_in1 + self.i_in2))
            with m.Case(Funct3.OR):
                m.d.comb += self.o_out.eq(self.i_in1 | self.i_in2)
            with m.Case(Funct3.AND):
                m.d.comb += self.o_out.eq(self.i_in1 & self.i_in2)
            with m.Case(Funct3.XOR):
                m.d.comb += self.o_out.eq(self.i_in1 ^ self.i_in2)
            with m.Case(Funct3.SLT):
                m.d.comb += self.o_out.eq(Mux(self.i_in1.as_signed()
                                            < self.i_in2.as_signed(), 0b1, 0b0))
            with m.Case(Funct3.SLTU):
                m.d.comb += self.o_out.eq(Mux(self.i_in1 < self.i_in2, 0b1, 0b0))
            with m.Case(Funct3.SLL):
                m.d.comb += self.o_out.eq(self.i_in1 << self.i_in2[:5])
            with m.Case(Funct3.SR):
                m.d.comb += self.o_out.eq(Mux(alt_func, self.i_in1.as_signed()
                                            >> self.i_in2[:5], self.i_in1 >> self.i_in2[:5]))

        m.d.comb += self.o_zero.eq(self.o_out == 0)

        return m


if __name__ == "__main__":
    alu = ALU(xlen=XLEN.RV32)

    with open("alu.v", "w") as file:
        file.write(verilog.convert(alu, ports=alu.ports()))
