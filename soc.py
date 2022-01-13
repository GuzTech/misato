from amaranth import *
from amaranth.cli import main_parser, main_runner
from amaranth.build import Platform
from amaranth.back import verilog
from amaranth.sim import Simulator, Settle
from amaranth_soc.wishbone import *

from amaranth.asserts import *

from cpu import *
from rom import ROM
from gpio import GPIO


class SoC(Elaboratable):
    def __init__(self, imem_init, formal=False):
        # Inputs

        # Outputs
        self.o_gpio    = Signal(32)
        self.o_trap    = Signal()
        self.o_ack     = Signal()
        self.o_addr    = Signal(5)

        # Configuration
        self.imem_init = imem_init
        self.formal    = formal

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        # imem = Memory(width=32, depth=128, init=self.imem_init)
        dmem = Memory(width=32, depth=128)

        m.submodules.cpu    = cpu    = MisatoWB(xlen=XLEN.RV32, formal=self.formal)
        m.submodules.gpio   = gpio   = GPIO()
        m.submodules.imem   = imem   = ROM(self.imem_init)
        m.submodules.dmem_r = dmem_r = dmem.read_port()
        m.submodules.dmem_w = dmem_w = dmem.write_port()

        # Connect cpu and instruction memory
        m.d.comb += cpu.ibus.connect(imem.arb.bus)

        # Connect cpu write to data bus arbiter
        with m.If(cpu.o_d_addr == 0x80):
            m.d.comb += gpio.i_w_en.eq(cpu.o_d_Wr)
            m.d.comb += dmem_w.en.eq(0)
        with m.Else():
            m.d.comb += gpio.i_w_en.eq(0)
            m.d.comb += dmem_w.en.eq(cpu.o_d_Wr)

        # Connect cpu to data bus
        m.d.comb += gpio.i_data.eq(cpu.o_d_data)
        m.d.comb += dmem_r.addr.eq(cpu.o_d_addr)
        m.d.comb += cpu.i_data.eq(dmem_r.data)
        m.d.comb += dmem_w.addr.eq(cpu.o_d_addr)
        m.d.comb += dmem_w.data.eq(cpu.o_d_data)

        m.d.comb += self.o_gpio.eq(gpio.o_data)
        m.d.comb += self.o_trap.eq(cpu.o_trap)
        m.d.comb += self.o_ack.eq(cpu.ibus.ack)
        m.d.comb += self.o_addr.eq(cpu.ibus.adr[2:7])

        return m


if __name__ == "__main__":
    # Knightrider program
    data = [
        # Setup
        RV32_I(imm= 0x01, rs1=0,        rd=2, funct3=Funct3.ADD),   # 0x00
        RV32_I(imm= 0x01, rs1=0,        rd=3, funct3=Funct3.ADD),   # 0x04
        RV32_I(imm= 0x80, rs1=0,        rd=5, funct3=Funct3.ADD),   # 0x08

        # Left setup
        RV32_I(imm= 0x00, rs1=0,        rd=1, funct3=Funct3.ADD),   # 0x0C
        #RV32_U(imm= 0x50,               rd=6, opcode=U_Instr.LUI), # 0x10
        RV32_I(imm= 0x00, rs1=0,        rd=6, funct3=Funct3.ADD),   # 0x10

        # Left
        RV32_I(imm= 0x01, rs1=1,        rd=1, funct3=Funct3.ADD),   # 0x14
        RV32_B(imm=-0x04, rs1=1, rs2=6,       funct3=Funct3.BLT),   # 0x18
        RV32_I(imm= 0x01, rs1=2,        rd=2, funct3=Funct3.SLL),   # 0x1C
        RV32_B(imm= 0x08, rs1=2, rs2=5,       funct3=Funct3.BEQ),   # 0x20
        RV32_J(imm=-0x18,               rd=0, opcode=J_Instr.JAL),  # 0x24

        # Right setup
        RV32_I(imm= 0x00, rs1=0,        rd=1, funct3=Funct3.ADD),   # 0x28
        # RV32_U(imm= 0x50,               rd=6, opcode=U_Instr.LUI),  # 0x2C
        RV32_I(imm= 0x00, rs1=0,        rd=6, funct3=Funct3.ADD),   # 0x2C

        # Right
        RV32_I(imm= 0x01, rs1=1,        rd=1, funct3=Funct3.ADD),   # 0x30
        RV32_B(imm=-0x04, rs1=1, rs2=6,       funct3=Funct3.BLT),   # 0x34
        RV32_I(imm= 0x01, rs1=2,        rd=2, funct3=Funct3.SR),    # 0x38
        RV32_B(imm=-0x30, rs1=2, rs2=3,       funct3=Funct3.BEQ),   # 0x3C
        RV32_J(imm=-0x18,               rd=0, opcode=J_Instr.JAL),  # 0x40
    ]

    top = Module()
    top.submodules.soc = soc = SoC(data, formal=True)
    parser = main_parser()
    args = parser.parse_args()
    main_runner(parser, args, top, ports=[soc.o_gpio])