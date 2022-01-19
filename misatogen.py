from fusesoc.capi2.generator import Generator

from amaranth import *
from amaranth.back import verilog

from isa import XLEN
from cpu import Misato

class MisatoGenerator(Generator):
    def run(self):
        cpu = Misato(xlen=XLEN.RV32, with_RVFI=False, formal=False)
        with open("cpu.v", "w") as file:
            file.write(verilog.convert(cpu, ports=cpu.ports()))

        files = [
            {"cpu.v" : {"file_type" : "verilogSource"}}]

        self.add_files(files)

g = MisatoGenerator()
g.run()
g.write()
