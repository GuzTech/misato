from nmigen import *
from nmigen_soc.wishbone import *
from nmigen_soc.memory import *

from ram import RAM
from rom import ROM


class Interconnect(Elaboratable):
    def __init__(self, rom: ROM, ram_words: int):
        self.dmux = Decoder(addr_width=32,
                            data_width=32,
                            alignment=0)
        self.imux = Decoder(addr_width=32,
                            data_width=32,
                            alignment=0)

        self.rom = rom
        # self.ram = RAM(ram_words)

        self.rom_dbus = self.rom.new_bus()
        # self.ram_dbus = self.ram.new_bus()
        self.dmux.add(self.rom_dbus, addr=0x0000_0000)
        # self.dmux.add(self.ram_dbus, addr=0x2000_0000)

        self.rom_ibus = self.rom.new_bus()
        # self.ram_ibus = self.ram.new_bus()
        self.imux.add(self.rom_ibus, addr=0x0000_0000)
        # self.imux.add(self.ram_ibus, addr=0x2000_0000)

    def elaborate(self, platform) -> Module:
        m = Module()

        m.submodules.dmux = self.dmux
        m.submodules.imux = self.imux
        m.submodules.rom  = self.rom
        # m.submodules.ram  = self.ram

        return m
