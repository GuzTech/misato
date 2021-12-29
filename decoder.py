from amaranth import *
from amaranth.cli import main_parser, main_runner
from amaranth.build import Platform
from amaranth.cli import main_parser, main_runner
from amaranth.back import verilog
from amaranth.sim import Simulator, Settle

from amaranth.asserts import Assert, Assume, Cover

from isa import *

from typing import List


class Decoder(Elaboratable):
    def __init__(self, xlen: XLEN, formal=False):
        # Inputs
        self.i_instr  = Signal(xlen.value)

        # Outputs
        self.o_format  = Signal(Format)
        self.o_opcode  = Signal(Opcode)
        self.o_LOAD    = Signal()
        self.o_STORE   = Signal()
        self.o_BRANCH  = Signal()
        self.o_JAL     = Signal()
        self.o_JALR    = Signal()
        self.o_OP_IMM  = Signal()
        self.o_OP      = Signal()
        self.o_SYSTEM  = Signal()
        self.o_AUIPC   = Signal()
        self.o_LUI     = Signal()
        self.o_jump    = Signal()
        self.o_u_instr = Signal()
        self.o_imm     = Signal(xlen.value)
        self.o_rs1     = Signal(5)
        self.o_rs2     = Signal(5)
        self.o_rd      = Signal(5)
        self.o_funct3  = Signal(Funct3)
        self.o_funct7  = Signal(Funct7)
        self.o_trap    = Signal()

        # Config
        self.xlen    = xlen.value
        self.formal  = formal

    def ports(self) -> List[Signal]:
        return [
            self.i_instr,
            self.o_format,
            self.o_opcode,
            self.o_LOAD,
            self.o_STORE,
            self.o_BRANCH,
            self.o_JAL,
            self.o_JALR,
            self.o_OP_IMM,
            self.o_OP,
            self.o_SYSTEM,
            self.o_AUIPC,
            self.o_LUI,
            self.o_jump,
            self.o_u_instr,
            self.o_imm,
            self.o_rs1,
            self.o_rs2,
            self.o_rd,
            self.o_funct3,
            self.o_funct7,
            self.o_trap,
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        # Internal signals
        opcode = Signal(Opcode)
        rd     = Signal(5)
        rs1    = Signal(5)
        rs2    = Signal(5)
        funct3 = Signal(Funct3)
        funct7 = Signal(Funct7)
        i_imm  = Signal(self.xlen)
        s_imm  = Signal(self.xlen)
        b_imm  = Signal(self.xlen)
        u_imm  = Signal(self.xlen)
        j_imm  = Signal(self.xlen)

        # Extract bit-fields from the instruction
        m.d.comb += [
            opcode.eq(self.i_instr[:7]),
            rd.eq(self.i_instr[7:12]),
            rs1.eq(self.i_instr[15:20]),
            rs2.eq(self.i_instr[20:25]),
            funct3.eq(Mux(self.o_u_instr, Funct3.ADD, self.i_instr[12:15])),
            funct7.eq(self.i_instr[25:]),
            i_imm.eq(Cat(self.i_instr[20:], Repl(self.i_instr[31], 20))),
            s_imm.eq(
                Cat(self.i_instr[7:12], self.i_instr[25:], Repl(self.i_instr[31], 20))),
            b_imm.eq(Cat(C(0), self.i_instr[8:12], self.i_instr[25:31],
                         self.i_instr[7], self.i_instr[31], Repl(self.i_instr[31], 19))),
            u_imm.eq(Cat(Repl(C(0), 12), self.i_instr[12:])),
            j_imm.eq(Cat(C(0), self.i_instr[21:31], self.i_instr[20],
                         self.i_instr[12:20], self.i_instr[31], Repl(self.i_instr[31], 11))),
            self.o_rs1.eq(rs1),
            self.o_rs2.eq(rs2),
            self.o_rd.eq(rd),
            self.o_funct3.eq(funct3),
            self.o_funct7.eq(funct7),
            self.o_opcode.eq(opcode),
        ]

        # Default values
        m.d.comb += [
            self.o_LOAD.eq(0),
            self.o_STORE.eq(0),
            self.o_BRANCH.eq(0),
            self.o_JALR.eq(0),
            self.o_JAL.eq(0),
            self.o_OP_IMM.eq(0),
            self.o_OP.eq(0),
            self.o_SYSTEM.eq(0),
            self.o_AUIPC.eq(0),
            self.o_LUI.eq(0),
            self.o_jump.eq(0),
            self.o_u_instr.eq(0),
            self.o_imm.eq(0),
            self.o_trap.eq(0),
        ]

        with m.Switch(opcode):
            with m.Case(Opcode.LOAD):
                m.d.comb += self.o_LOAD.eq(1)
                m.d.comb += self.o_format.eq(Format.I_type)
                m.d.comb += self.o_imm.eq(i_imm)
            with m.Case(Opcode.STORE):
                m.d.comb += self.o_STORE.eq(1)
                m.d.comb += self.o_format.eq(Format.S_type)
                m.d.comb += self.o_imm.eq(s_imm)
            with m.Case(Opcode.BRANCH):
                m.d.comb += self.o_BRANCH.eq(1)
                m.d.comb += self.o_format.eq(Format.B_type)
                m.d.comb += self.o_imm.eq(b_imm)
            with m.Case(Opcode.JALR):
                m.d.comb += self.o_JALR.eq(1)
                m.d.comb += self.o_format.eq(Format.I_type)
                m.d.comb += self.o_imm.eq(i_imm)
                m.d.comb += self.o_jump.eq(1)
            with m.Case(Opcode.JAL):
                m.d.comb += self.o_JAL.eq(1)
                m.d.comb += self.o_format.eq(Format.J_type)
                m.d.comb += self.o_imm.eq(j_imm)
                m.d.comb += self.o_jump.eq(1)
            with m.Case(Opcode.OP_IMM):
                m.d.comb += self.o_OP_IMM.eq(1)
                m.d.comb += self.o_format.eq(Format.I_type)
                m.d.comb += self.o_imm.eq(i_imm)
            with m.Case(Opcode.OP):
                m.d.comb += self.o_OP.eq(1)
                m.d.comb += self.o_format.eq(Format.R_type)
                m.d.comb += self.o_imm.eq(0)
            with m.Case(Opcode.SYSTEM):
                m.d.comb += self.o_SYSTEM.eq(1)
                m.d.comb += self.o_format.eq(Format.I_type)
            with m.Case(Opcode.AUIPC):
                m.d.comb += self.o_AUIPC.eq(1)
                m.d.comb += self.o_format.eq(Format.U_type)
                m.d.comb += self.o_imm.eq(u_imm)
                m.d.comb += self.o_u_instr.eq(1)
            with m.Case(Opcode.LUI):
                m.d.comb += self.o_LUI.eq(1)
                m.d.comb += self.o_format.eq(Format.U_type)
                m.d.comb += self.o_imm.eq(u_imm)
                m.d.comb += self.o_u_instr.eq(1)
            with m.Default():
                m.d.comb += [
                    self.o_format.eq(Format.Unknown_type),
                    self.o_LOAD.eq(0),
                    self.o_STORE.eq(0),
                    self.o_BRANCH.eq(0),
                    self.o_JALR.eq(0),
                    self.o_JAL.eq(0),
                    self.o_OP_IMM.eq(0),
                    self.o_OP.eq(0),
                    self.o_SYSTEM.eq(0),
                    self.o_AUIPC.eq(0),
                    self.o_LUI.eq(0),
                    self.o_jump.eq(0),
                    self.o_u_instr.eq(0),
                    self.o_imm.eq(0),
                    self.o_trap.eq(1)
                ]
        
        #
        # Formal verification
        #

        if self.formal:
            # Check if LOAD instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.LOAD):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b1),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == i_imm),
                    Assert(self.o_format  == Format.I_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_LOAD == 0b0)

            # Check if STORE instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.STORE):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b1),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == s_imm),
                    Assert(self.o_format  == Format.S_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_STORE == 0b0)

            # Check if BRANCH instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.BRANCH):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b1),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == b_imm),
                    Assert(self.o_format  == Format.B_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_BRANCH == 0b0)

            # Check if JALR instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.JALR):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b1),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b1),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == i_imm),
                    Assert(self.o_format  == Format.I_type),
                ]
            with m.Else():
                m.d.comb += Assert(self.o_JALR == 0b0)
                m.d.comb += Assert(self.o_jump == 0b0)

            # Check if JAL instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.JAL):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b1),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b1),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == j_imm),
                    Assert(self.o_format  == Format.J_type),
                ]
            with m.Else():
                m.d.comb += Assert(self.o_JAL  == 0b0)
                m.d.comb += Assert(self.o_jump == 0b0)

            # Check if OP-IMM instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.OP_IMM):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b1),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == i_imm),
                    Assert(self.o_format  == Format.I_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_OP_IMM == 0b0)

            # Check if OP instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.OP):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b1),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_format  == Format.R_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_OP == 0b0)

            # Check if SYSTEM instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.SYSTEM):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b1),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_format  == Format.I_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_SYSTEM == 0b0)

            # Check if AUIPC instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.AUIPC):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b1),
                    Assert(self.o_LUI     == 0b0),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == u_imm),
                    Assert(self.o_format  == Format.U_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_AUIPC == 0b0)

            # Check if AUIPC instructions are correctly decoded
            with m.If(self.o_opcode == Opcode.AUIPC):
                m.d.comb += [
                    Assert(self.o_LOAD    == 0b0),
                    Assert(self.o_STORE   == 0b0),
                    Assert(self.o_BRANCH  == 0b0),
                    Assert(self.o_JALR    == 0b0),
                    Assert(self.o_JAL     == 0b0),
                    Assert(self.o_OP_IMM  == 0b0),
                    Assert(self.o_OP      == 0b0),
                    Assert(self.o_SYSTEM  == 0b0),
                    Assert(self.o_AUIPC   == 0b0),
                    Assert(self.o_LUI     == 0b1),
                    Assert(self.o_jump    == 0b0),
                    Assert(self.o_u_instr == 0b0),
                    Assert(self.o_trap    == 0b0),
                    Assert(self.o_imm     == u_imm),
                    Assert(self.o_format  == Format.U_type)
                ]
            with m.Else():
                m.d.comb += Assert(self.o_LUI == 0b0)

        return m


if __name__ == "__main__":
    formal = True

    m = Module()
    m.submodules.decoder = decoder = Decoder(xlen=XLEN.RV32, formal=formal)

    if not formal:
        def bench():
            # ADDI R31 = R1 + (-1)
            yield decoder.i_instr.eq(0b1111_1111_1111_00001_000_11111_0010011)
            yield Settle()

            assert ((yield decoder.o_format) == (Format.I_type.value))
            assert (yield decoder.i_type)
            assert not (yield decoder.o_trap)

            # yield decoder.instr.eq(-1)
            # yield Settle()
            # assert (yield decoder.trap)

        sim = Simulator(decoder)
        sim.add_process(bench)
        with sim.write_vcd("decoder.vcd"):
            sim.run()
        
        with open("decoder.v", "w") as file:
            file.write(verilog.convert(m, ports=decoder.ports()))
    else:
        parser = main_parser()
        args = parser.parse_args()
        main_runner(parser, args, decoder, ports=decoder.ports())
