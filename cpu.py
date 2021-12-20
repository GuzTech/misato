from typing import List

from amaranth import *
from amaranth.build import Platform
from amaranth.back import verilog
from amaranth.sim import Simulator, Settle
from amaranth_soc.wishbone import *
from amaranth_boards.ulx3s import *

from isa import *
from decoder import Decoder
from alu import ALU
from branch import Branch, BInsn
from rom import ROM
from ram import RAM
from interconnect import Interconnect


class Misato(Elaboratable):
    def __init__(self,
                 xlen: XLEN,
                 with_RVFI=False):
        # Configuration
        self.xlen     = xlen                # Number of bits
        self.rvfi     = with_RVFI           # Enable RVFI for formal verification

        # Inputs
        self.i_instr  = Signal(32)          # Fetched instruction (I-mem)
        self.i_data   = Signal(xlen.value)  # Fetched data (D-mem)
        self.i_stall  = Signal()            # Stall the CPU

        # Outputs
        self.o_i_addr = Signal(xlen.value)  # Instruction memory address
        self.o_d_addr = Signal(xlen.value)  # Data memory address
        self.o_d_data = Signal(xlen.value)  # Data memory value to be written
        self.o_trap   = Signal()            # CPU encountered an issue
        self.o_reg    = Signal(xlen.value)  # Debug

        # RVFI Interface (optional)
        if with_RVFI:
            self.rvfi_valid     = Signal()
            self.rvfi_order     = Signal(64)
            self.rvfi_insn      = Signal(xlen.value)
            self.rvfi_trap      = Signal()
            self.rvfi_halt      = Signal()
            self.rvfi_intr      = Signal()
            self.rvfi_mode      = Signal(2)
            self.rvfi_ixl       = Signal(2)
            self.rvfi_rs1_addr  = Signal(5)
            self.rvfi_rs2_addr  = Signal(5)
            self.rvfi_rs1_rdata = Signal(xlen.value)
            self.rvfi_rs2_rdata = Signal(xlen.value)
            self.rvfi_rd_addr   = Signal(5)
            self.rvfi_rd_wdata  = Signal(xlen.value)
            self.rvfi_mem_addr  = Signal(xlen.value)
            self.rvfi_pc_rdata  = Signal(xlen.value)
            self.rvfi_pc_wdata  = Signal(xlen.value)
            self.rvfi_mem_rmask = Signal(xlen.value//8)
            self.rvfi_mem_wmask = Signal(xlen.value//8)
            self.rvfi_mem_rdata = Signal(xlen.value)
            self.rvfi_mem_wdata = Signal(xlen.value)

    def ports(self) -> List[Signal]:
        signals = [
            self.i_instr,
            self.i_data,
            self.i_stall,
            self.o_i_addr,
            self.o_d_addr,
            self.o_d_data,
            self.o_trap,
            self.o_reg
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

        # Register file
        regfile = Memory(width=self.xlen.value, depth=32)
        m.submodules.rp1 = rp1 = regfile.read_port()
        m.submodules.rp2 = rp2 = regfile.read_port()
        m.submodules.wp  = wp  = regfile.write_port()

        ###########
        # Signals #
        ###########
        branch_taken = Signal()
        todo = Signal()
        m.d.comb += branch_taken.eq(0)
        m.d.comb += todo.eq(0)

        #
        # Instruction fetch stage signals (IF)
        #
        pc_IF           = Signal(self.xlen.value, reset=-4)   # Current program counter value
        pc_next_IF      = Signal(self.xlen.value)   # Next program counter value
        pc_p4_IF        = Signal(self.xlen.value)   # Current program counter value + 4
        instr_IF        = Signal(self.xlen.value,   # Fetched instruction
                                 reset=NOP)

        #
        # Instruction decode stage signals (ID)
        #
        pc_ID           = Signal(self.xlen.value)   # Program counter value
        instr_ID        = Signal(self.xlen.value)   # Fetched instruction
        r1_ID           = Signal(self.xlen.value)   # Source register #1 value
        r2_ID           = Signal(self.xlen.value)   # Source register #2 value

        # Instruction decoder signals
        format_ID       = Signal(Format)            # Instruction format
        r_type_ID       = Signal()                  # R type instruction
        i_type_ID       = Signal()                  # I type instruction
        u_type_ID       = Signal()                  # U type instruction
        s_type_ID       = Signal()                  # S type instruction
        b_type_ID       = Signal()                  # B type instruction
        j_type_ID       = Signal()                  # J type instruction
        imm_ID          = Signal(self.xlen.value)   # Immediate value
        rs1_ID          = Signal(5)                 # Source register #1
        rs2_ID          = Signal(5)                 # Source register #2
        rd_ID           = Signal(5)                 # Destination register
        funct3_ID       = Signal(Funct3)            # Funct3 field
        funct7_ID       = Signal(Funct7)            # Funct7 field
        u_instr_ID      = Signal(U_Type)            # U type instruction
        trap_ID         = Signal()                  # Illegal instruction

        #
        # Execute stage signals (EX)
        #
        pc_EX           = Signal(self.xlen.value)   # Program counter value
        r1_EX           = Signal(self.xlen.value)   # Source register #1 value
        r2_EX           = Signal(self.xlen.value)   # Source register #2 value
        rd_EX           = Signal(5)                 # Destination register
        imm_EX          = Signal(self.xlen.value)   # Immediate value
        branch_addr_EX  = Signal(self.xlen.value)   # Branch address
        alu_out_EX      = Signal(self.xlen.value)   # ALU output
        zero_EX         = Signal()                  # ALU result is zero

        #
        #  Memory stage signals (MEM)
        #
        r2_MEM          = Signal(self.xlen.value)   # Source register #2 value
        rd_MEM          = Signal(5)                 # Destination register
        branch_addr_MEM = Signal(self.xlen.value)   # Branch address
        data_MEM        = Signal(self.xlen.value)   # Data value read from data memory
        alu_out_MEM     = Signal(self.xlen.value)   # ALU output
        zero_MEM        = Signal()                  # ALU result is zero

        # 
        # Write-back stage signals (WB)
        #
        rd_WB           = Signal(5)                 # Destination register
        data_WB         = Signal(self.xlen.value)   # Data value to be written to the register file
        alu_out_WB      = Signal(self.xlen.value)   # ALU output
        data_val_WB     = Signal(self.xlen.value)   # WB stage data value

        ##########
        # Stages #
        ##########

        #
        # Instruction fetch stage (IF)
        #
        m.d.comb += pc_p4_IF.eq(pc_IF + 4)
        m.d.comb += pc_next_IF.eq(Mux(branch_taken, branch_addr_MEM, pc_p4_IF))

        m.d.sync += pc_IF.eq(pc_next_IF)
        m.d.comb += self.o_i_addr.eq(pc_IF)

        #
        # Instruction decode stage (ID)
        #
        m.d.sync += pc_ID.eq(pc_IF)
        m.d.sync += instr_ID.eq(instr_IF)

        m.d.comb += decoder.instr.eq(instr_ID)
        m.d.comb += [
            format_ID .eq(decoder.format),
            r_type_ID .eq(decoder.r_type),
            i_type_ID .eq(decoder.i_type),
            s_type_ID .eq(decoder.s_type),
            u_type_ID .eq(decoder.u_type),
            b_type_ID .eq(decoder.b_type),
            j_type_ID .eq(decoder.j_type),
            imm_ID    .eq(decoder.imm),
            rs1_ID    .eq(decoder.rs1),
            rs2_ID    .eq(decoder.rs2),
            rd_ID     .eq(decoder.rd),
            funct3_ID .eq(decoder.funct3),
            funct7_ID .eq(decoder.funct7),
            u_instr_ID.eq(decoder.u_instr),
            trap_ID   .eq(decoder.trap)
        ]

        m.d.comb += rp1.addr.eq(rs1_ID)
        m.d.comb += rp2.addr.eq(rs2_ID)
        m.d.comb += wp.addr.eq(rd_WB)
        m.d.comb += wp.data.eq(data_val_WB)

        #
        # Execute stage (EX)
        #
        m.d.sync += pc_EX.eq(pc_ID)
        m.d.sync += r1_EX.eq(r1_ID)
        m.d.sync += r2_EX.eq(r2_ID)
        m.d.sync += rd_EX.eq(rd_ID)
        m.d.sync += imm_EX.eq(imm_ID)

        m.d.comb += branch_addr_EX.eq(pc_EX + Cat(0b0, imm_EX[0:-1]))
        m.d.comb += alu.i_in1.eq(r1_EX)
        m.d.comb += alu.i_in2.eq(Mux(todo, imm_EX, r2_EX))
        m.d.comb += alu_out_EX.eq(alu.o_out)
        m.d.comb += zero_EX.eq(alu.o_zero)

        #
        # Memory stage (MEM)
        #
        m.d.sync += r2_MEM.eq(r2_EX)
        m.d.sync += rd_MEM.eq(rd_EX)
        m.d.sync += branch_addr_MEM.eq(branch_addr_EX)
        m.d.sync += alu_out_MEM.eq(alu_out_EX)
        m.d.sync += zero_MEM.eq(zero_EX)

        m.d.comb += self.o_d_addr.eq(alu_out_MEM)
        m.d.comb += self.o_d_data.eq(r2_MEM)
        m.d.comb += data_MEM.eq(self.i_data)

        #
        # Write-back stage (WB)
        #
        m.d.sync += rd_WB.eq(rd_MEM)
        m.d.sync += data_WB.eq(data_MEM)
        m.d.sync += alu_out_WB.eq(alu_out_MEM)

        m.d.comb += data_val_WB.eq(Mux(todo, data_WB, alu_out_WB))

        return m
 

if __name__ == "__main__":
    top = Module()
    top.submodules.cpu = cpu = Misato(xlen=XLEN.RV32, with_RVFI=False)

    mem = {
        0x0000_0000: RV32_I(imm= 0x01, rs1=0, rd =1, funct3=Funct3.ADD),
        0x0000_0004: RV32_I(imm= 0x01, rs1=1, rd =1, funct3=Funct3.SLL),
        0x0000_0008: RV32_B(imm= 0x1C, rs1=1, rs2=2, funct3=Funct3.BEQ),
        0x0000_000C: RV32_I(imm= 0x01, rs1=2, rd =2, funct3=Funct3.ADD),
        0x0000_0010: RV32_J(imm=-0x08, rd =0, opcode=J_Instr.JAL),
        0x0000_0024: RV32_J(imm=-0x24, rd =0, opcode=J_Instr.JAL),
    }

    with top.Switch(cpu.o_i_addr):
        for addr, data in mem.items():
            with top.Case(addr):
                top.d.sync += cpu.i_instr.eq(data)
                top.d.sync += cpu.i_stall.eq(0)
        with top.Default():
            top.d.sync += cpu.i_instr.eq(0)
            top.d.sync += cpu.i_stall.eq(1)

    def bench():
        yield
        assert not (yield cpu.o_trap)

        for _ in range(len(mem)):
            yield
            assert not (yield cpu.o_trap)
        
        # for _ in range(30):
        #     yield
        #     assert not (yield cpu.o_trap)
        
    sim = Simulator(top)
    sim.add_clock(1e-6)
    sim.add_sync_process(bench)
    with sim.write_vcd("cpu.vcd"):
        sim.run()

    with open("cpu.v", "w") as file:
        file.write(verilog.convert(cpu, ports=cpu.ports()))