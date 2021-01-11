from nmigen import *
from nmigen.build import Platform
from nmigen.back import verilog
from nmigen.sim import Simulator, Settle
from nmigen_soc.wishbone import *

from isa import *
from decoder import Decoder
from alu import ALU

from typing import List


class CPU(Elaboratable):
    def __init__(self,
                 xlen: XLEN,
                 with_RVFI=False):
        # Inputs
        self.imem = Interface(addr_width=xlen.value, data_width=xlen.value)
        self.dmem = Interface(addr_width=xlen.value, data_width=xlen.value)

        # Outputs
        self.trap = Signal()

        # Configuration
        self.xlen = xlen

        # RVFI Interface (optional)
        if with_RVFI:
            self.rvfi_valid = Signal()
            self.rvfi_order = Signal(64)
            self.rvfi_insn = Signal()
            self.rvfi_trap = Signal()
            self.rvfi_halt = Signal()
            self.rvfi_intr = Signal()
            self.rvfi_mode = Signal(2)
            self.rvfi_ixl = Signal(2)
            self.rvfi_rs1_addr = Signal(5)
            self.rvfi_rs2_addr = Signal(5)
            self.rvfi_rs1_rdata = Signal(xlen.value)
            self.rvfi_rs2_rdata = Signal(xlen.value)
            self.rvfi_rd_addr = Signal(5)
            self.rvfi_rd_wdata = Signal(xlen.value)
            self.rvfi_mem_addr = Signal(xlen.value)
            self.rvfi_mem_rmask = Signal(xlen.value/8)
            self.rvfi_mem_wmask = Signal(xlen.value/8)
            self.rvfi_mem_rdata = Signal(xlen.value)
            self.rvfi_mem_wdata = Signal(xlen.value)

    def ports(self) -> List[Signal]:
        return [
            self.imem.adr,
            self.imem.dat_r,
            self.imem.sel,
            self.imem.cyc,
            self.imem.stb,
            self.imem.we,
            self.imem.ack,
            self.dmem.adr,
            self.dmem.dat_r,
            self.dmem.sel,
            self.dmem.cyc,
            self.dmem.stb,
            self.dmem.we,
            self.dmem.ack,
            self.trap
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.submodules.decoder = decoder = Decoder(self.xlen)
        m.submodules.alu     = alu     = ALU(self.xlen)

        # Register file
        regfile = Memory(width=self.xlen.value, depth=32, init=[(i+3) for i in range(32)])
        m.submodules.rp1 = rp1 = regfile.read_port()
        m.submodules.rp2 = rp2 = regfile.read_port()
        m.submodules.wp  = wp  = regfile.write_port()

        # Decode stage signals
        rs1_value_d = Signal(self.xlen.value)
        rs2_value_d = Signal(self.xlen.value)
        funct3_x    = Signal(Funct3)
        funct7_x    = Signal(Funct7)
        imm_x       = Signal(self.xlen.value)
        bubble_next = Signal()
        bubble_d    = Signal(reset=1)

        # Execute stage signals
        rs1_value_x = Signal(self.xlen.value)
        rs2_value_x = Signal(self.xlen.value)
        pc_x      = Signal(self.xlen.value)
        rd_x      = Signal(5)
        format_x  = Signal(Format)
        u_instr_x = Signal()
        alu_out_wb = Signal(self.xlen.value)
        bubble_x  = Signal(reset=1)

        # Write-back stage signals
        pc_wb     = Signal(self.xlen.value)
        rd_wb     = Signal(5)
        format_wb = Signal(Format)
        bubble_wb = Signal(reset=1)

        # Program counter
        pc      = Signal(self.xlen.value)
        pc_next = Signal(self.xlen.value)

        m.d.comb += pc_next.eq(pc + 4)
        m.d.sync += pc.eq(Mux((format_x == Format.J_type), alu.out, Mux(bubble_next, pc, pc_next)))
        # m.d.comb += self.imem.adr.eq(pc)

        # When we're at the execution stage, then we know what the target address
        # of the J-type instruction is.
        m.d.comb += self.imem.adr.eq(Mux((format_x == Format.J_type), alu.out, pc))
        m.d.comb += self.imem.cyc.eq(1)

        with m.If(~bubble_next):
            m.d.comb += self.imem.stb.eq(1)
        with m.Else():
            m.d.comb == self.imem.stb.eq(0)

        ################
        # Decode stage #
        ################
        m.d.comb += [
            decoder.instr.eq(self.imem.dat_r),
            self.trap.eq(decoder.trap)
        ]

        m.d.sync += format_x.eq(decoder.format)
        m.d.sync += funct3_x.eq(decoder.funct3)
        m.d.sync += funct7_x.eq(decoder.funct7)
        m.d.sync += rd_x.eq(decoder.rd)
        m.d.sync += imm_x.eq(decoder.imm)
        m.d.sync += u_instr_x.eq(decoder.u_instr)
        
        # with m.If((decoder.format == Format.J_type) | (format_x == Format.J_type)):
        #     with m.If(format_wb != Format.J_type):
        #         m.d.sync += bubble_d.eq(1)
        #     with m.Else():
        #         m.d.sync += bubble_d.eq(0)
        # with m.Else():
        #     m.d.sync += bubble_d.eq(0)

        # with m.If((decoder.format == Format.J_type) | (format_x == Format.J_type)):
        #     with m.If(format_wb != Format.J_type):
        #         m.d.comb += bubble_next.eq(1)
        #     with m.Else():
        #         m.d.comb += bubble_next.eq(0)
        # with m.Else():
        #     m.d.comb += bubble_next.eq(0)

        with m.If(decoder.format == Format.J_type):
            # If this is the very first stage that
            # is processing the J-type instruction
            with m.If(format_x != Format.J_type):
                # The insert a bubble so that we don't
                # process anything after the J-type
                # instruction.
                m.d.comb += bubble_next.eq(1)
            with m.Else():
                m.d.comb += bubble_next.eq(0)
        with m.Else():
            m.d.comb += bubble_next.eq(0)

        m.d.sync += bubble_d.eq(bubble_next)

        with m.If(~bubble_d):
            with m.Switch(decoder.format):
                with m.Case(Format.R_type):
                    m.d.comb += rp1.addr.eq(decoder.rs1)
                    # m.d.sync += rs1_value_x.eq(rs1_value_RAW)
                    m.d.comb += rp2.addr.eq(decoder.rs2)
                    # m.d.sync += rs2_value_x.eq(rs2_value_RAW)
                with m.Case(Format.I_type, Format.S_type):
                    m.d.comb += rp1.addr.eq(decoder.rs1)
                    # m.d.sync += rs1_value_x.eq(rs1_value_RAW)
                    # m.d.sync += rs2_value_x.eq(decoder.imm)
                    m.d.sync += rs2_value_d.eq(decoder.imm)
                with m.Case(Format.U_type):
                    # Determine if the U-type instruction is
                    # LUI (add 0) or AUIPC (add PC).
                    m.d.sync += rs1_value_d.eq(Mux(decoder.u_instr, pc, 0))
                    m.d.sync += rs2_value_d.eq(decoder.imm)                    
                with m.Case(Format.B_type, Format.J_type):
                    m.d.sync += rs1_value_d.eq(pc)
                    m.d.sync += rs2_value_d.eq(decoder.imm)


        #################
        # Execute stage #
        #################

        # Read-after-write (RAW) hazard prevention
        # 
        # We have a three-stage pipeline, where we
        # check if any of the current source registers
        # match the destination register of the previous
        # one or two instructions. If there is a match,
        # then we forward the result so that we don't
        # have to stall the pipeline. 
        rs1_value_RAW = Signal(self.xlen.value)
        rs2_value_RAW = Signal(self.xlen.value)

        with m.If(decoder.rs1 == rd_x):
            m.d.sync += rs1_value_RAW.eq(alu.out)
        with m.Elif(decoder.rs1 == rd_wb):
            m.d.sync += rs1_value_RAW.eq(alu_out_wb)
        # with m.Else():
        #     m.d.comb += rs1_value_RAW.eq(rp1.data)

        with m.If(decoder.rs2 == rd_x):
            m.d.comb += rs2_value_RAW.eq(alu.out)
        with m.Elif(decoder.rs2 == rd_wb):
            m.d.comb += rs2_value_RAW.eq(alu_out_wb)
        with m.Else():
            m.d.comb += rs2_value_RAW.eq(rp2.data)

        m.d.sync += pc_x.eq(pc)
        m.d.sync += bubble_x.eq(bubble_d)

        # ALU signals
        with m.If(~bubble_x):
            with m.Switch(format_x):
                with m.Case(Format.R_type):
                    m.d.comb += alu.in1.eq(rs1_value_x)
                    m.d.comb += alu.in2.eq(rs2_value_x)
                    m.d.comb += alu.funct3.eq(funct3_x)
                    m.d.comb += alu.funct7.eq(funct7_x)
                with m.Case(Format.I_type):
                    # m.d.comb += alu.in1.eq(rs1_value_x)
                    # m.d.comb += alu.in1.eq(rp1.data)
                    
                    with m.If(rd_wb == rd_x):
                        m.d.comb += alu.in1.eq(rs1_value_RAW)
                    with m.Else():
                        m.d.comb += alu.in1.eq(rp1.data)

                    m.d.comb += alu.in2.eq(imm_x)
                    m.d.comb += alu.funct3.eq(funct3_x)
                    # m.d.comb += alu.funct7.eq(0)
                with m.Case(Format.S_type, Format.U_type):
                    m.d.comb += alu.in1.eq(rs1_value_x)
                    m.d.comb += alu.in2.eq(rs2_value_x)
                    m.d.comb += alu.funct3.eq(Funct3.ADD)
                    # m.d.comb += alu.funct7.eq(0)
                # with m.Case(Format.U_type):
                #     m.d.comb += alu.in1.eq(rs1_value)
                #     m.d.comb += alu.in2.eq(rs2_value)
                #     m.d.comb += alu.funct3.eq(Funct3.ADD)
                #     m.d.comb += alu.funct7.eq(0)
                with m.Case(Format.J_type):
                    m.d.comb += alu.in1.eq(rs1_value_x)
                    m.d.comb += alu.in2.eq(rs2_value_x)
                    m.d.comb += alu.funct3.eq(Funct3.ADD)

        ####################
        # Write-back stage #
        ####################

        m.d.sync += pc_wb.eq(pc_x)
        m.d.sync += rd_wb.eq(rd_x)
        m.d.sync += format_wb.eq(format_x)
        m.d.sync += alu_out_wb.eq(alu.out)

        m.d.sync += bubble_wb.eq(bubble_x)

        with m.If((~bubble_wb) & (rd_wb != 0)):
            with m.Switch(format_wb):
                with m.Case(Format.R_type, Format.I_type):
                    m.d.comb += wp.en.eq(1)
                    m.d.comb += wp.data.eq(alu_out_wb)
                    m.d.comb += wp.addr.eq(rd_wb)
                with m.Case(Format.J_type):
                    # JAL stores PC+4 in the destination
                    # register, however pc_x already has PC+4
                    # stored in it, so don't add 4 to it.
                    m.d.comb += wp.en.eq(1)
                    m.d.comb += wp.data.eq(pc_wb)
                    m.d.comb += wp.addr.eq(rd_x)
                with m.Default():
                    m.d.comb += wp.en.eq(0)
        with m.Else():
            m.d.comb += wp.en.eq(0)

        return m


if __name__ == "__main__":
    top = Module()

    top.submodules.cpu = cpu = CPU(xlen=XLEN.RV32)

    mem = {
        0x0000_0000: 0b1111_1111_1111_00001_000_00010_0010011, # ADDI R2 = R1 + (-1)
        0x0000_0004: 0b0000_0000_0010_00010_000_00010_0010011, # ADDI R2 = R2 + 2
        0x0000_0008: 0b0000_0000_0010_00001_000_00001_0010011, # ADDI R1 = R1 + 2
        0x0000_000C: 0b0000_0000_0010_00010_000_00010_0010011, # ADDI R2 = R2 + 2
        0x0000_0010: 0b1111_1110_1101_1111_1111_00000_1101111, # JAL R0, -16
    }

    with top.If(cpu.imem.cyc & cpu.imem.stb):
        with top.Switch(cpu.imem.adr):
            for addr, data in mem.items():
                with top.Case(addr):
                    top.d.sync += cpu.imem.dat_r.eq(data)
                    top.d.sync += cpu.imem.ack.eq(1)
            with top.Default():
                # NOP
                top.d.sync += cpu.imem.dat_r.eq(0b0000_0000_0000_00000_000_00000_0010011)
                top.d.sync += cpu.imem.ack.eq(1)
    with top.Else():
        cpu.imem.ack.eq(0)

    def bench():
        yield
        assert not (yield cpu.trap)

        for _ in range(len(mem)):
            yield
        
        for _ in range(10):
            yield
        
    sim = Simulator(top)
    sim.add_clock(1e-6)
    sim.add_sync_process(bench)
    with sim.write_vcd("cpu.vcd"):
        sim.run()

    with open("cpu.v", "w") as file:
        file.write(verilog.convert(cpu, ports=cpu.ports()))
