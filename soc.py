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
    def __init__(self, imem_init):
        # Inputs

        # Outputs
        self.o_gpio    = Signal(32)
        self.o_trap    = Signal()

        # Configuration
        self.imem_init = imem_init

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        imem = Memory(width=32, depth=128, init=self.imem_init)
        dmem = Memory(width=32, depth=128)

        m.submodules.cpu    = cpu    = MisatoWB(xlen=XLEN.RV32)
        m.submodules.gpio   = gpio   = GPIO()
        # m.submodules.imem_r = imem_r = imem.read_port()
        m.submodules.imem   = imem   = ROM(self.imem_init)
        m.submodules.dmem_r = dmem_r = dmem.read_port()
        m.submodules.dmem_w = dmem_w = dmem.write_port()

        # Connect cpu and instruction memory
        # m.d.comb += cpu.i_instr.eq(imem_r.data)
        # m.d.comb += imem_r.addr.eq(cpu.o_i_addr[2:])
        m.d.comb += cpu.ibus.connect(imem.arb.bus)

        # m.d.comb += cpu.i_ack.eq(1)

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

        return m
