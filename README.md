# Misato RISC-V RV32I CPU

Misato is a RISC-V CPU that supports the RV32I instruction set. It has not won any awards (yet) but it has [FuseSoC support](https://github.com/olofk/misato/tree/fusesoc)! It is formally verified in that each instruction performs what and when it is supposed to perform. Currently it requires separate, single-cycle instruction and data memories, but in the interconnect branch I'm working on a Wishbone Classic interface (which does not pass formal verification yet). The core is written in [Amaranth HDL](https://github.com/amaranth-lang/amaranth).

## Status
All RV32I instructions have been implemented except for FENCE, ECALL, and EBREAK.

It has been implemented with a [65 nm PDK](https://bitlog.it/20220118_asic_roundup_of_open_source_riscv_cpu_cores.html) and the [Skywater 130 nm PDK](https://twitter.com/OlofKindgren/status/1483914264033341441) using [OpenLANE](https://github.com/The-OpenROAD-Project/OpenLane) by the [award-winning](https://riscv.org/blog/2018/12/risc-v-softcpu-contest-highlights/) [Olof Kindgren](https://twitter.com/OlofKindgren).