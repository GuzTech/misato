import argparse

from nmigen import *
from nmigen.build import Platform
from nmigen.back import verilog
from nmigen.sim import Simulator, Settle
from nmigen_soc.wishbone import *
from nmigen_boards.ulx3s import *

from cpu import *
from rom import ROM

if __name__ == "__main__":
    data = []

    with open("programs/hex.bin", "rb") as f:
        bindata = f.read()

    for i in range(len(bindata) // 4):
        w = bindata[4*i : 4*i+4]
        data.append(int("%02x%02x%02x%02x" % (w[3], w[2], w[1], w[0]), 16))

    variants = {
        '12F': ULX3S_12F_Platform,
        '25F': ULX3S_25F_Platform,
        '45F': ULX3S_45F_Platform,
        '85F': ULX3S_85F_Platform
    }

    # Figure out which FPGA variant we want to target...
    parser = argparse.ArgumentParser()
    parser.add_argument('variant', choices=variants.keys())
    args = parser.parse_args()

    platform = variants[args.variant]()

    top = Module()
    top.submodules.cpu = cpu = CPU(xlen=XLEN.RV32, with_RVFI=False)

    leds = [platform.request("led", 0),
            platform.request("led", 1),
            platform.request("led", 2),
            platform.request("led", 3),
            platform.request("led", 4),
            platform.request("led", 5),
            platform.request("led", 6),
            platform.request("led", 7)]

    for i in range(len(leds)):
        top.d.comb += leds[i].eq(cpu.reg[i+20])

    top.submodules.imem = mem = ROM(data)

    top.d.comb += cpu.imem.connect(mem.wbus)

    platform.build(top, do_program=True, nextpnr_opts="--timing-allow-fail")
