from amaranth import *
from amaranth.back import verilog
from amaranth.build import Platform

from isa import *

from typing import List


class Forwarding(Elaboratable):
    def __init__(self, xlen: XLEN):
        # Inputs
        self.i_rs1_EX     = Signal(5)   # Source register #1 in EX stage
        self.i_rs2_EX     = Signal(5)   # Source register #2 in EX stage
        self.i_reg_wr_MEM = Signal()    # Register write in the MEM stage
        self.i_reg_wr_WB  = Signal()    # Register write in the MEM stage
        self.i_rd_MEM     = Signal(5)   # Destination register in MEM stage
        self.i_rd_WB      = Signal(5)   # Destination register in WB stage
        
        # Outputs
        self.o_fwdA   = Signal(2)   # Selector signal MUX of A input of ALU
        self.o_fwdB   = Signal(2)   # Selector signal MUX of B input of ALU

    def ports(self) -> List[Signal]:
        return [
            self.i_rs1_EX,
            self.i_rs2_EX,
            self.i_rd_EX,
            self.i_rd_MEM,
            self.o_fwdA,
            self.o_fwdB,
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        # Default values
        m.d.comb += [
            self.o_fwdA.eq(0b00),
            self.o_fwdB.eq(0b00),
        ]

        # Local signals
        ex_hazard  = Signal()
        ex_rs1_hazard = Signal()
        ex_rs2_hazard = Signal()
        mem_hazard = Signal()
        mem_rs1_hazard = Signal()
        mem_rs2_hazard = Signal()

        m.d.comb += ex_hazard.eq(self.i_reg_wr_MEM & (self.i_rd_MEM != 0))
        m.d.comb += ex_rs1_hazard.eq(self.i_rd_MEM == self.i_rs1_EX)
        m.d.comb += ex_rs2_hazard.eq(self.i_rd_MEM == self.i_rs2_EX)
        m.d.comb += mem_hazard.eq(self.i_reg_wr_WB & (self.i_rd_WB != 0))
        m.d.comb += mem_rs1_hazard.eq(self.i_rd_WB == self.i_rs1_EX)
        m.d.comb += mem_rs2_hazard.eq(self.i_rd_WB == self.i_rs2_EX)

        # EX hazard
        with m.If(ex_hazard):
            with m.If(ex_rs1_hazard):
                m.d.comb += self.o_fwdA.eq(0b10)
            with m.If(ex_rs2_hazard):
                m.d.comb += self.o_fwdB.eq(0b10)

        # MEM hazard
        with m.If(mem_hazard):
            with m.If(mem_rs1_hazard & ~(ex_hazard & ex_rs1_hazard)):
                m.d.comb += self.o_fwdA.eq(0b01)
            with m.If(mem_rs2_hazard & ~(ex_hazard & ex_rs2_hazard)):
                m.d.comb += self.o_fwdB.eq(0b01)

        return m


if __name__ == "__main__":
    fwd = Forwarding(xlen=XLEN.RV32)

    with open("fwd.v", "w") as file:
        file.write(verilog.convert(fwd, ports=fwd.ports()))
