from typing import List

from nmigen import *
from nmigen.build import Platform
from nmigen.back import verilog
from nmigen.sim import Simulator, Settle
from nmigen_soc.wishbone import *
from nmigen_boards.ulx3s import *

from isa import *
from decoder import Decoder
from alu import ALU
from branch import Branch, BInsn
from rom import ROM
from ram import RAM
from interconnect import Interconnect


class CPU(Elaboratable):
    def __init__(self,
                 xlen: XLEN,
                 with_RVFI=False):
        # Inputs
        self.ibus  = Interface(addr_width=xlen.value, data_width=xlen.value)
        self.dbus  = Interface(addr_width=xlen.value, data_width=xlen.value)
        self.stall = Signal()

        # Outputs
        self.trap = Signal()
        self.reg  = Signal(32)

        # Configuration
        self.xlen = xlen
        self.rvfi = with_RVFI

        # RVFI Interface (optional)
        if with_RVFI:
            self.rvfi_valid = Signal()
            self.rvfi_order = Signal(64)
            self.rvfi_insn = Signal(xlen.value)
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
            self.rvfi_pc_rdata = Signal(xlen.value)
            self.rvfi_pc_wdata = Signal(xlen.value)
            self.rvfi_mem_rmask = Signal(xlen.value//8)
            self.rvfi_mem_wmask = Signal(xlen.value//8)
            self.rvfi_mem_rdata = Signal(xlen.value)
            self.rvfi_mem_wdata = Signal(xlen.value)

    def ports(self) -> List[Signal]:
        signals = [
            self.ibus.adr,
            self.ibus.dat_r,
            self.ibus.sel,
            self.ibus.cyc,
            self.ibus.stb,
            self.ibus.we,
            self.ibus.ack,
            self.dbus.adr,
            self.dbus.dat_r,
            self.dbus.sel,
            self.dbus.cyc,
            self.dbus.stb,
            self.dbus.we,
            self.dbus.ack,
            self.stall,
            self.trap
        ]

        if self.rvfi:
            signals += [
                self.rvfi_valid,
                self.rvfi_order,
                self.rvfi_insn,
                self.rvfi_trap,
                self.rvfi_halt,
                self.rvfi_intr,
                self.rvfi_mode,
                self.rvfi_ixl,
                self.rvfi_rs1_addr,
                self.rvfi_rs2_addr,
                self.rvfi_rs1_rdata,
                self.rvfi_rs2_rdata,
                self.rvfi_rd_addr,
                self.rvfi_rd_wdata,
                self.rvfi_pc_rdata,
                self.rvfi_pc_wdata,
                self.rvfi_mem_addr,
                self.rvfi_mem_rmask,
                self.rvfi_mem_wmask,
                self.rvfi_mem_rdata,
                self.rvfi_mem_wdata
            ]

        return signals

    def elaborate(self, platform: Platform) -> Module:
        m = Module()

        m.submodules.decoder = decoder = Decoder(self.xlen)
        m.submodules.alu     = alu     = ALU(self.xlen)
        m.submodules.branch  = branch  = Branch(self.xlen)

        # Register file
        regfile = Memory(width=self.xlen.value, depth=32)
        m.submodules.rp1 = rp1 = regfile.read_port()
        m.submodules.rp2 = rp2 = regfile.read_port()
        m.submodules.wp  = wp  = regfile.write_port()

        m.submodules.rp3 = rp3 = regfile.read_port()
        m.d.comb += rp3.addr.eq(2)
        m.d.comb += self.reg.eq(rp3.data)

        # Instruction fetch stage signals
        insn     = Signal(self.xlen.value, reset=NOP)
        cyc      = Signal()
        stb      = Signal()
        addr     = Signal(self.xlen.value, reset=0)
        cyc_prev = Signal(reset=0)
        stb_prev = Signal(reset=0)

        # Decode stage signals
        rs1_value_d = Signal(self.xlen.value)
        rs2_value_d = Signal(self.xlen.value)
        funct3_x    = Signal(Funct3)
        funct7_x    = Signal(Funct7)
        imm_x       = Signal(self.xlen.value)
        stall_next  = Signal()
        stall_d     = Signal(reset=1)

        # Execute stage signals
        rs1_RAW     = Signal()
        rs2_RAW     = Signal()
        pc_x        = Signal(self.xlen.value)
        rd_x        = Signal(5)
        format_x    = Signal(Format)
        u_instr_x   = Signal()
        alu_out_wb  = Signal(self.xlen.value)
        branch_x    = Signal()
        stall_x     = Signal(reset=1)

        # Write-back stage signals
        pc_wb     = Signal(self.xlen.value)
        format_wb = Signal(Format)
        stall_wb  = Signal(reset=1)

        # Wishbone interface
        # m.d.sync += self.ibus.cyc.eq(cyc)
        # m.d.sync += self.ibus.stb.eq(stb)
        # m.d.sync += self.ibus.adr.eq(addr)

        # Program counter
        pc      = Signal(self.xlen.value)
        pc_next = Signal(self.xlen.value)

        jump_x = Signal()
        m.d.comb += jump_x.eq(format_x == Format.J_type)

        branch_addr_known = Signal()

        with m.If(jump_x):
            m.d.comb += branch_addr_known.eq(1)
            m.d.comb += pc_next.eq(alu.out + 4)
        with m.Elif(branch_x):
            m.d.comb += branch_addr_known.eq(1)
            m.d.comb += pc_next.eq(alu.out)
        with m.Else():
            m.d.comb += branch_addr_known.eq(0)
            m.d.comb += pc_next.eq(pc + 4)

        # m.d.sync += pc.eq(Mux(stall_next, pc, pc_next))
        m.d.sync += pc.eq(pc_next)

        # When we're at the execution stage, then we know what
        # the target address of the J-type instruction is.
        m.d.comb += self.ibus.adr.eq(Mux(jump_x, alu.out, pc))
        # m.d.comb += self.ibus.adr.eq(pc)
        m.d.comb += self.ibus.cyc.eq(1)
        # m.d.comb += cyc.eq(1)

        with m.If((~stall_next)):
            m.d.comb += self.ibus.stb.eq(1)
            # m.d.comb += self.ibus.sel.eq(3)
            # m.d.comb += stb.eq(1)
        with m.Else():
            m.d.comb == self.ibus.stb.eq(0)
            # m.d.comb += self.ibus.sel.eq(0)
            # m.d.comb == stb.eq(0)
        
        m.d.sync += cyc_prev.eq(self.ibus.cyc)
        m.d.sync += stb_prev.eq(self.ibus.stb)

        # This might be dangerous, because we only check
        # if we issued a instruction fetch the previous
        # cycle instead of checking if we have an outstanding
        # instruction fetch.
        with m.If((format_x == Format.B_type) & (branch_x)):
            m.d.sync += insn.eq(NOP)
        with m.Elif(decoder.format == Format.J_type):
            m.d.sync += insn.eq(NOP)
        with m.Elif(cyc_prev & self.ibus.stb & self.ibus.ack):
            # TODO: Try to use self.ibus.dat_r combinatorially
            # whenever possible, but when self.ibus.ack is 0,
            # then use the registered insn.
            m.d.sync += insn.eq(self.ibus.dat_r)

        ################
        # Decode stage #
        ################

        m.d.comb += [
            decoder.instr.eq(insn),
            self.trap.eq(decoder.trap),
        ]

        m.d.sync += [
            self.dbus.adr.eq(0),
            self.dbus.we.eq(0),
            self.dbus.cyc.eq(0),
            self.dbus.stb.eq(0),
            self.dbus.dat_w.eq(0),
            self.dbus.sel.eq(0)
        ]

        m.d.sync += format_x.eq(Mux(branch_x, Format.I_type, decoder.format))
        m.d.sync += funct3_x.eq(decoder.funct3)
        m.d.sync += funct7_x.eq(decoder.funct7)
        m.d.sync += rd_x.eq(Mux(branch_x | jump_x | stall_x, 0, decoder.rd))
        m.d.sync += imm_x.eq(decoder.imm)
        m.d.sync += u_instr_x.eq(decoder.u_instr)

        with m.If(decoder.format == Format.J_type):
            # If this is the very first stage that
            # is processing the J-type instruction
            with m.If(format_x != Format.J_type):
                # The insert a bubble so that we don't
                # decode anything while calculating
                # the jump address.
                m.d.comb += stall_next.eq(1)
            with m.Else():
                m.d.comb += stall_next.eq(0)
        with m.Elif(branch_x):
            m.d.comb += stall_next.eq(1)
        # This creates a combinatorial loop
        # with m.Elif(self.ibus.cyc & self.ibus.stb & (~self.ibus.ack)):
        # This does not, but hangs because of no progress
        # with m.Elif(~self.ibus.ack):
        #     m.d.comb += stall_next.eq(1)
        with m.Else():
            m.d.comb += stall_next.eq(0)

        m.d.sync += stall_d.eq(stall_next)

        with m.If((~stall_d) & (~branch_x)):
            with m.Switch(decoder.format):
                with m.Case(Format.R_type):
                    m.d.comb += rp1.addr.eq(decoder.rs1)
                    m.d.comb += rp2.addr.eq(decoder.rs2)
                    m.d.sync += rs1_value_d.eq(0)
                    m.d.sync += rs2_value_d.eq(0)
                with m.Case(Format.I_type, Format.S_type):
                    m.d.comb += rp1.addr.eq(decoder.rs1)
                    m.d.comb += rp2.addr.eq(0)
                    m.d.sync += rs1_value_d.eq(0)
                    m.d.sync += rs2_value_d.eq(decoder.imm)
                with m.Case(Format.U_type):
                    # Determine if the U-type instruction is
                    # LUI (add 0) or AUIPC (add PC).
                    m.d.sync += rs1_value_d.eq(Mux(decoder.u_instr, pc, 0))
                    m.d.sync += rs2_value_d.eq(decoder.imm)

                    m.d.comb += rp1.addr.eq(0)
                    m.d.comb += rp2.addr.eq(0)
                with m.Case(Format.B_type):
                    m.d.sync += rs1_value_d.eq(pc - 8)
                    m.d.sync += rs2_value_d.eq(decoder.imm)

                    m.d.comb += rp1.addr.eq(decoder.rs1)
                    m.d.comb += rp2.addr.eq(decoder.rs2)
                with m.Case(Format.J_type):
                    m.d.sync += rs1_value_d.eq(pc - 8)
                    m.d.sync += rs2_value_d.eq(decoder.imm)

                    m.d.comb += rp1.addr.eq(0)
                    m.d.comb += rp2.addr.eq(0)
        with m.Else():
            m.d.comb += [
                rp1.addr.eq(0),
                rp2.addr.eq(0),
            ]
            m.d.sync += [
                rs1_value_d.eq(0),
                rs2_value_d.eq(0),
            ]


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
            m.d.sync += rs1_RAW.eq(1)
        with m.Else():
            m.d.sync += rs2_value_RAW.eq(0)
            m.d.sync += rs1_RAW.eq(0)

        with m.If(decoder.rs2 == rd_x):
            m.d.sync += rs2_value_RAW.eq(alu.out)
            m.d.sync += rs2_RAW.eq(1)
        with m.Else():
            m.d.sync += rs2_value_RAW.eq(0)
            m.d.sync += rs2_RAW.eq(0)

        m.d.comb += branch_x.eq(Mux(format_x == Format.B_type, branch.take_branch, 0))

        m.d.sync += pc_x.eq(pc)
        m.d.sync += stall_x.eq(stall_d)

        # Default values
        m.d.comb += branch.in1.eq(0)
        m.d.comb += branch.in2.eq(0)
        m.d.comb += branch.br_insn.eq(BInsn.BNE)

        m.d.comb += alu.in1.eq(0)
        m.d.comb += alu.in2.eq(0)
        m.d.comb += alu.funct3.eq(Funct3.ADD)

        m.d.comb += wp.data.eq(0)
        m.d.comb += wp.addr.eq(0)
        m.d.comb += wp.en.eq(0)

        # ALU signals
        with m.If(~stall_x):
            with m.Switch(format_x):
                with m.Case(Format.R_type):
                    with m.If(rs1_RAW):
                        m.d.comb += alu.in1.eq(rs1_value_RAW)
                    with m.Else():
                        m.d.comb += alu.in1.eq(rp1.data)
                    with m.If(rs2_RAW):
                        m.d.comb += alu.in2.eq(rs2_value_RAW)
                    with m.Else():
                        m.d.comb += alu.in2.eq(rp2.data)

                    m.d.comb += alu.funct3.eq(funct3_x)
                    m.d.comb += alu.funct7.eq(funct7_x)

                    # For cleanness sake, prefer this...
                    with m.If(rd_x != 0):
                        m.d.comb += wp.data.eq(alu.out)
                        m.d.comb += wp.addr.eq(rd_x)
                        m.d.comb += wp.en.eq(1)
                    # over this...
                    # m.d.comb += wp.data.eq(alu.out)
                    # m.d.comb += wp.addr.eq(rd_x)
                    # m.d.comb += wp.en.eq(rd_x != 0)
                with m.Case(Format.I_type):
                    with m.If(rs1_RAW):
                        m.d.comb += alu.in1.eq(rs1_value_RAW)
                    with m.Else():
                        m.d.comb += alu.in1.eq(rp1.data)

                    m.d.comb += alu.in2.eq(rs2_value_d)
                    m.d.comb += alu.funct3.eq(funct3_x)

                    # For cleanness sake, prefer this...
                    with m.If(rd_x != 0):
                        m.d.comb += wp.data.eq(alu.out)
                        m.d.comb += wp.addr.eq(rd_x)
                        m.d.comb += wp.en.eq(1)
                    # over this...
                    # m.d.comb += wp.data.eq(alu.out)
                    # m.d.comb += wp.addr.eq(rd_x)
                    # m.d.comb += wp.en.eq(rd_x != 0)
                with m.Case(Format.S_type, Format.U_type):
                    with m.If(rs1_RAW):
                        m.d.comb += alu.in1.eq(rs1_value_RAW)
                    with m.Else():
                        m.d.comb += alu.in1.eq(rp1.data)

                    m.d.comb += alu.in2.eq(rs2_value_d)
                    m.d.comb += alu.funct3.eq(Funct3.ADD)

                    # For cleanness sake, prefer this...
                    with m.If(rd_x != 0):
                        m.d.comb += wp.data.eq(alu.out)
                        m.d.comb += wp.addr.eq(rd_x)
                        m.d.comb += wp.en.eq(1)
                    # over this...
                    # m.d.comb += wp.data.eq(alu.out)
                    # m.d.comb += wp.addr.eq(rd_x)
                    # m.d.comb += wp.en.eq(rd_x != 0)
                with m.Case(Format.B_type):
                    m.d.comb += alu.in1.eq(rs1_value_d)
                    m.d.comb += alu.in2.eq(rs2_value_d)
                    m.d.comb += alu.funct3.eq(Funct3.ADD)

                    with m.If(rs1_RAW):
                        m.d.comb += branch.in1.eq(rs1_value_RAW)
                    with m.Else():
                        m.d.comb += branch.in1.eq(rp1.data)

                    with m.If(rs2_RAW):
                        m.d.comb += branch.in2.eq(rs2_value_RAW)
                    with m.Else():
                        m.d.comb += branch.in2.eq(rp2.data)

                    m.d.comb += branch.br_insn.eq(funct3_x)
                with m.Case(Format.J_type):
                    m.d.comb += alu.in1.eq(rs1_value_d)
                    m.d.comb += alu.in2.eq(rs2_value_d)
                    m.d.comb += alu.funct3.eq(Funct3.ADD)
                    
                    # For cleanness sake, prefer this...
                    with m.If(rd_x != 0):
                        m.d.comb += wp.data.eq(pc_wb)
                        m.d.comb += wp.addr.eq(rd_x)
                        m.d.comb += wp.en.eq(1)
                    # over this...
                    # m.d.comb += wp.data.eq(pc_wb)
                    # m.d.comb += wp.addr.eq(rd_x)
                    # m.d.comb += wp.en.eq(rd_x != 0)

        ####################
        # Write-back stage #
        ####################

        m.d.sync += pc_wb.eq(pc_x)
        m.d.sync += format_wb.eq(format_x)
        m.d.sync += alu_out_wb.eq(alu.out)

        m.d.sync += stall_wb.eq(stall_x)

        # with m.If((~stall_wb)):
        #     with m.Switch(format_wb):
        #         with m.Case(Format.S_type):

        #         # with m.Case(Format.R_type, Format.I_type, Format.U_type):
        #         #     m.d.comb += wp.en.eq(1)
        #         #     m.d.comb += wp.data.eq(alu_out_wb)
        #         #     m.d.comb += wp.addr.eq(rd_wb)
        #         # with m.Case(Format.J_type):
        #         #     # JAL stores PC+4 in the destination
        #         #     # register, however pc_x already has PC+4
        #         #     # stored in it, so don't add 4 to it.
        #         #     m.d.comb += wp.en.eq(1)
        #         #     m.d.comb += wp.data.eq(pc_wb)
        #         #     m.d.comb += wp.addr.eq(rd_x)
        #         with m.Default():
        #             m.d.comb += wp.en.eq(0)
        # with m.Else():
        #     m.d.comb += wp.en.eq(0)

        # RVFI
        if self.rvfi:
            m.d.sync += self.rvfi_valid.eq((~ResetSignal()) & (~self.trap) & (~stall_wb))
            m.d.sync += self.rvfi_trap.eq(self.trap)
            m.d.sync += self.rvfi_insn.eq(self.ibus.dat_r)
            m.d.sync += self.rvfi_pc_rdata.eq(pc)
            m.d.sync += self.rvfi_pc_wdata.eq(pc_x)
            m.d.sync += self.rvfi_rs1_addr.eq(decoder.rs1)
            m.d.sync += self.rvfi_rs2_addr.eq(decoder.rs2)
            m.d.sync += self.rvfi_rs1_rdata.eq(rs1_value_d)
            m.d.sync += self.rvfi_rs2_rdata.eq(rs2_value_d)
            m.d.sync += self.rvfi_rd_addr.eq(decoder.rd)
            m.d.sync += self.rvfi_rd_wdata.eq(alu_out_wb)
            m.d.sync += self.rvfi_mem_addr.eq(self.ibus.adr)
            m.d.sync += self.rvfi_mem_rdata.eq(self.ibus.dat_r)
            
            m.d.comb += self.rvfi_mode.eq(0)
            m.d.comb += self.rvfi_ixl.eq(1)

        return m


if __name__ == "__main__":
    data = []

    with open("programs/hex.bin", "rb") as f:
        bindata = f.read()

    for i in range(len(bindata) // 4):
        w = bindata[4*i : 4*i+4]
        data.append(int("%02x%02x%02x%02x" % (w[3], w[2], w[1], w[0]), 16))

    top = Module()
    top.submodules.cpu = cpu = CPU(xlen=XLEN.RV32, with_RVFI=False)
    # top.submodules.imem = mem = ROM(data)
    top.submodules.interconnect = itcnt = Interconnect(ROM(data), 512)

    top.d.comb += cpu.ibus.connect(itcnt.imux.bus)
    top.d.comb += cpu.dbus.connect(itcnt.dmux.bus)

    # top.d.comb += cpu.ibus.connect(mem.new_bus())

    def bench():
        yield
        assert not (yield cpu.trap)

        for _ in range(len(data)):
            yield
            assert not (yield cpu.trap)
        
        for _ in range(400):
            yield
            assert not (yield cpu.trap)
        
    sim = Simulator(top)
    sim.add_clock(1e-6)
    sim.add_sync_process(bench)
    with sim.write_vcd("cpu.vcd"):
        sim.run()

    with open("cpu.v", "w") as file:
        file.write(verilog.convert(cpu, ports=cpu.ports()))