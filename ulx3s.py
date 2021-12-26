import argparse
import subprocess
import sys

from amaranth import *
from amaranth.build import Platform
from amaranth.back import verilog
from amaranth.sim import Simulator, Settle
from amaranth_soc.wishbone import *
from amaranth_boards.ulx3s import *

from cpu import *
from rom import ROM

if __name__ == "__main__":
    variants = {
        '12F': ULX3S_12F_Platform,
        '25F': ULX3S_25F_Platform,
        '45F': ULX3S_45F_Platform,
        '85F': ULX3S_85F_Platform
    }

    # Figure out which FPGA variant we want to target...
    parser = argparse.ArgumentParser()
    parser.add_argument('variant', choices=variants.keys())
    parser.add_argument('program')
    args = parser.parse_args()

    platform = variants[args.variant]()

    subprocess.run(["riscv64-elf-as", "-c", "programs/%s.s" % args.program, "-o", "programs/a.out"])
    subprocess.run(["riscv64-elf-objdump", "-d", "programs/a.out"])
    subprocess.run(["riscv64-elf-objcopy", "-O", "binary", "programs/a.out", "programs/hex.bin"])

    data = []

    with open("programs/hex.bin", "rb") as f:
        bindata = f.read()

    for i in range(len(bindata) // 4):
        w = bindata[4*i : 4*i+4]
        data.append(int("%02x%02x%02x%02x" % (w[3], w[2], w[1], w[0]), 16))

    top = Module()
    top.submodules.cpu = cpu = Misato(xlen=XLEN.RV32, with_RVFI=False)

    leds = [platform.request("led", 0),
            platform.request("led", 1),
            platform.request("led", 2),
            platform.request("led", 3),
            platform.request("led", 4),
            platform.request("led", 5),
            platform.request("led", 6),
            platform.request("led", 7)]

    for i in range(len(leds)):
        top.d.comb += leds[i].eq(cpu.o_reg[i])
 
    # top.submodules.imem = mem = ROM(data)
    # top.d.comb += cpu.ibus.connect(mem.arb.bus)

    imem = Memory(width=32, depth=128, init=data)
    top.submodules.imem_r = imem_r = imem.read_port()
    top.d.comb += cpu.i_instr.eq(imem_r.data)
    top.d.comb += imem_r.addr.eq(cpu.o_i_addr[2:])

    dmem = Memory(width=32, depth=128)
    top.submodules.dmem_r = dmem_r = dmem.read_port()
    top.submodules.dmem_w = dmem_w = dmem.write_port()
    top.d.comb += dmem_r.addr.eq(cpu.o_d_addr[2:])
    top.d.comb += cpu.i_data.eq(dmem_r.data)
    top.d.comb += dmem_w.addr.eq(cpu.o_d_addr[2:])
    top.d.comb += dmem_w.en.eq(cpu.o_d_Wr)
    top.d.comb += dmem_w.data.eq(cpu.o_d_data)

    # We define our own "sync" clock domain so that we
    # can access the reset signal for simulation.
    # This does decouple the actual clock signal from
    # our clock domain, so we have to manually connect
    # the global clock to the clock domain. This does
    # decrease the maximum clock speed a bit for some
    # reason, so remove it when not needed.
    sync = ClockDomain()
    top.domains += sync
    clk = platform.request(platform.default_clk)
    top.d.comb += sync.clk.eq(clk)

    def bench():
        yield sync.rst.eq(1)
        yield
        yield sync.rst.eq(0)
        yield
        assert not (yield cpu.o_trap)
        
        for _ in range(200):
            yield
            assert not (yield cpu.o_trap)
        
    sim = Simulator(top)
    sim.add_clock(1e-6)
    sim.add_sync_process(bench)
    with sim.write_vcd("cpu.vcd"):
        sim.run()

    platform.build(top, do_program=True, nextpnr_opts="--timing-allow-fail")
