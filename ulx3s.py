import argparse
import subprocess
import sys

from amaranth import *
from amaranth_boards.ulx3s import *

from soc import SoC

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

    # Select the correct platform
    platform = variants[args.variant]()

    # Assemble the program and extract the hex representation
    subprocess.run(["riscv64-elf-as", "-c", "programs/%s.s" % args.program, "-o", "programs/a.out"])
    subprocess.run(["riscv64-elf-objdump", "-d", "programs/a.out"])
    subprocess.run(["riscv64-elf-objcopy", "-O", "binary", "programs/a.out", "programs/hex.bin"])

    # Fill the data list with the instructions (as hex)
    data = []
    with open("programs/hex.bin", "rb") as f:
        bindata = f.read()

    for i in range(len(bindata) // 4):
        w = bindata[4*i : 4*i+4]
        data.append(int("%02x%02x%02x%02x" % (w[3], w[2], w[1], w[0]), 16))

    # Build the SoC
    top = Module()
    top.submodules.soc = soc = SoC(imem_init=data)

    # Connect the low 8-bit outputs of the GPIO
    # to the LEDs on the ULX3S.
    leds = [platform.request("led", 0),
            platform.request("led", 1),
            platform.request("led", 2),
            platform.request("led", 3),
            platform.request("led", 4),
            platform.request("led", 5),
            platform.request("led", 6),
            platform.request("led", 7)]

    for i in range(len(leds)):
        top.d.comb += leds[i].eq(soc.o_gpio[i])

    platform.build(top, do_program=True, nextpnr_opts="--timing-allow-fail")
