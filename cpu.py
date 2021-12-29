from typing import List

from amaranth import *
from amaranth.cli import main_parser, main_runner
from amaranth.build import Platform
from amaranth.back import verilog
from amaranth.sim import Simulator, Settle
from amaranth_soc.wishbone import *
from amaranth_boards.ulx3s import *

from amaranth.asserts import *

from isa import *
from decoder import Decoder
from alu import ALU
from branch import Branch, BInsn
from forwarding import Forwarding
from rom import ROM
from ram import RAM
from interconnect import Interconnect


class Misato(Elaboratable):
    def __init__(self,
                 xlen: XLEN,
                 with_RVFI=False,
                 formal=False):
        # Configuration
        self.xlen     = xlen                # Number of bits
        self.rvfi     = with_RVFI           # Enable RVFI for formal verification
        self.formal   = formal              # Enable formal verification

        # Inputs
        self.i_instr  = Signal(32)          # Fetched instruction (I-mem)
        self.i_data   = Signal(xlen.value)  # Fetched data (D-mem)
        self.i_stall  = Signal()            # Stall the CPU

        # Outputs
        self.o_i_addr = Signal(xlen.value)  # Instruction memory address
        self.o_i_en   = Signal()            # Read enable
        self.o_d_addr = Signal(xlen.value)  # Data memory address
        self.o_d_data = Signal(xlen.value)  # Data memory value to be written
        self.o_d_Rd   = Signal()            # Read from data memory
        self.o_d_Wr   = Signal()            # Write to data memory
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
            self.o_i_en,
            self.o_d_addr,
            self.o_d_data,
            self.o_d_Rd,
            self.o_d_Wr,
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
        m.submodules.branch  = branch  = Branch(self.xlen)
        m.submodules.fwd     = fwd     = Forwarding(self.xlen)

        # Register file
        regfile = Memory(width=self.xlen.value, depth=32)
        m.submodules.rp1 = rp1 = regfile.read_port()
        m.submodules.rp2 = rp2 = regfile.read_port()
        m.submodules.wp  = wp  = regfile.write_port()

        ###########
        # Signals #
        ###########

        #
        # Instruction fetch stage signals (IF)
        #
        pc_IF            = Signal(self.xlen.value)  # Current program counter value
        pc_next_IF       = Signal(self.xlen.value)  # Next program counter value
        pc_p4_IF         = Signal(self.xlen.value)  # Current program counter value + 4

        # Branch stall signals
        bubble_count_IF  = Signal(2)                # How many NOPs to insert
        insert_bubble_IF = Signal()                 # Insert a NOP instruction
        preload_next_IF  = Signal()                 # When to preload next instruction

        #
        # Instruction decode stage signals (ID)
        #
        pc_ID            = Signal(self.xlen.value)  # Program counter value
        pc_p4_ID         = Signal(self.xlen.value)  # Current program counter value + 4
        instr_ID         = Signal(self.xlen.value)  # Fetched instruction

        # Instruction decoder signals
        format_ID        = Signal(Format)           # Instruction format
        opcode_ID        = Signal(Opcode)           # Instruction opcode
        r_type_ID        = Signal()                 # R type instruction
        i_type_ID        = Signal()                 # I type instruction
        u_type_ID        = Signal()                 # U type instruction
        s_type_ID        = Signal()                 # S type instruction
        b_type_ID        = Signal()                 # B type instruction
        j_type_ID        = Signal()                 # J type instruction
        imm_ID           = Signal(self.xlen.value)  # Immediate value
        rs1_ID           = Signal(5)                # Source register #1
        rs2_ID           = Signal(5)                # Source register #2
        rd_ID            = Signal(5)                # Destination register
        funct3_ID        = Signal(Funct3)           # Funct3 field
        funct7_ID        = Signal(Funct7)           # Funct7 field
        u_instr_ID       = Signal(U_Type)           # U type instruction
        trap_ID          = Signal()                 # Illegal instruction
        pc_mod_instr_ID  = Signal()                 # PC modifying instruction
        JALR_instr_ID    = Signal()                 # Instruction is JALR

        # Control signals
        pc_source_C_ID   = Signal()                 # pc_next source selector
        reg_write_C_ID   = Signal()                 # Write to register file
        mem_write_C_ID   = Signal()                 # Write to data memory
        mem_read_C_ID    = Signal()
        mem_to_reg_C_ID  = Signal()                 # Mem or ALU result to register
        alu_source_C_ID  = Signal()                 # ALU input #2 source

        #
        # Execute stage signals (EX)
        #
        pc_EX            = Signal(self.xlen.value)  # Program counter value
        pc_p4_EX         = Signal(self.xlen.value)  # Current program counter value + 4
        rs1_EX           = Signal(5)                # Source register #1
        rs2_EX           = Signal(5)                # Source register #2
        r1_EX            = Signal(self.xlen.value)  # Source register #1 value
        r2_EX            = Signal(self.xlen.value)  # Source register #2 value
        rd_EX            = Signal(5)                # Destination register
        opcode_EX        = Signal(Opcode)           # Instruction opcode
        funct3_EX        = Signal(Funct3)           # Funct3 field
        funct7_EX        = Signal(Funct7)           # Funct7 field
        imm_EX           = Signal(self.xlen.value)  # Immediate value
        branch_addr_EX   = Signal(self.xlen.value)  # Branch address
        branch_addr_t_EX = Signal(self.xlen.value)  # Temporary branch address
        alu_out_EX       = Signal(self.xlen.value)  # ALU output
        zero_EX          = Signal()                 # ALU result is zero
        reg_write_C_EX   = Signal()                 # Write to register file
        mem_to_reg_C_EX  = Signal()                 # Mem or ALU result to register
        mem_read_C_EX    = Signal()                 # Read from data memory
        mem_write_C_EX   = Signal()                 # Write to data memory
        alu_source_C_EX  = Signal()                 # ALU input #2 source
        b_type_EX        = Signal()                 # B type instruction
        j_type_EX        = Signal()                 # J type instruction
        u_type_EX        = Signal()                 # U type instruction
        in1_fwd_EX       = Signal(self.xlen.value)  # EX stage ALU/BR fowarded value #1
        in1_EX           = Signal(self.xlen.value)  # EX stage ALU/BR value #1
        in2_fwd_EX       = Signal(self.xlen.value)  # EX stage ALU/BR fowarded value #2
        in2_EX           = Signal(self.xlen.value)  # EX stage ALU/BR value #2
        JALR_instr_EX    = Signal()                 # Instruction is JALR

        #
        #  Memory stage signals (MEM)
        #
        r2_MEM           = Signal(self.xlen.value)  # Source register #2 value
        rd_MEM           = Signal(5)                # Destination register
        branch_addr_MEM  = Signal(self.xlen.value)  # Branch address
        data_MEM         = Signal(self.xlen.value)  # Data value read from data memory
        alu_out_MEM      = Signal(self.xlen.value)  # ALU output
        zero_MEM         = Signal()                 # ALU result is zero
        take_branch_MEM  = Signal()                 # Branch unit output
        pc_source_C_MEM  = Signal()                 # pc_next source selector
        reg_write_C_MEM  = Signal()                 # Write to register file
        mem_to_reg_C_MEM = Signal()                 # Mem or ALU result to register
        mem_read_C_MEM   = Signal()                 # Read from data memory
        mem_write_C_MEM  = Signal()                 # Write to data memory
        b_type_MEM       = Signal()                 # B type instruction
        j_type_MEM       = Signal()                 # J type instruction
        valid_branch_MEM = Signal()                 # Take the branch
        valid_jump_MEM   = Signal()                 # Take the jump

        # 
        # Write-back stage signals (WB)
        #
        rd_WB            = Signal(5)                # Destination register
        data_WB          = Signal(self.xlen.value)  # Data value to be written to the register file
        alu_out_WB       = Signal(self.xlen.value)  # ALU output
        data_val_WB      = Signal(self.xlen.value)  # WB stage data value
        reg_write_C_WB   = Signal()                 # Write to register file
        mem_to_reg_C_WB  = Signal()                 # Mem or ALU result to register

        ##########
        # Stages #
        ##########

        #
        # Instruction fetch stage (IF)
        #
        m.d.comb += pc_p4_IF.eq(pc_IF + 4)
        m.d.comb += pc_next_IF.eq(Mux(pc_source_C_MEM,
                                      branch_addr_MEM,
                                      Mux(pc_mod_instr_ID | preload_next_IF, pc_IF, pc_p4_IF)))

        m.d.sync += pc_IF.eq(pc_next_IF)
        m.d.comb += preload_next_IF.eq(bubble_count_IF > 1)
        m.d.comb += self.o_i_en.eq(~pc_mod_instr_ID)
        m.d.comb += self.o_i_addr.eq(pc_IF)

        with m.If(pc_mod_instr_ID):
            m.d.sync += bubble_count_IF.eq(3)
        with m.Elif(insert_bubble_IF):
            m.d.sync += bubble_count_IF.eq(bubble_count_IF - 1)
        m.d.comb += insert_bubble_IF.eq(bubble_count_IF > 0)

        #
        # Instruction decode stage (ID)
        #
        m.d.sync += pc_ID.eq(pc_IF)
        m.d.sync += pc_p4_ID.eq(pc_p4_IF)
        m.d.comb += instr_ID.eq(Mux(insert_bubble_IF, NOP, self.i_instr))

        m.d.comb += pc_mod_instr_ID.eq(b_type_ID | j_type_ID)

        m.d.comb += JALR_instr_ID.eq(opcode_ID == Opcode.JALR)

        m.d.comb += decoder.instr.eq(instr_ID)
        m.d.comb += [
            format_ID .eq(decoder.format),
            opcode_ID .eq(decoder.opcode),
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

        m.d.comb += reg_write_C_ID.eq(
            ((opcode_ID == Opcode.LOAD) |
             (opcode_ID == Opcode.OP) |
             (opcode_ID == Opcode.OP_IMM) |
             u_type_ID |
             j_type_ID) &
            (rd_ID != 0)
        )
        m.d.comb += mem_write_C_ID.eq(decoder.s_type)
        m.d.comb += mem_read_C_ID.eq(opcode_ID == Opcode.LOAD)
        m.d.comb += mem_to_reg_C_ID.eq(opcode_ID == Opcode.LOAD)
        m.d.comb += alu_source_C_ID.eq(
            (opcode_ID == Opcode.OP_IMM) |
            u_type_ID | s_type_ID
        )

        m.d.comb += rp1.addr.eq(rs1_ID)
        m.d.comb += rp2.addr.eq(rs2_ID)
        m.d.comb += wp.addr.eq(rd_WB)
        m.d.comb += wp.data.eq(data_val_WB)
        m.d.comb += wp.en.eq(reg_write_C_WB)

        #
        # Execute stage (EX)
        #
        m.d.sync += [
            pc_EX.eq(pc_ID),
            pc_p4_EX.eq(pc_p4_ID),
            rs1_EX.eq(rs1_ID),
            rs2_EX.eq(rs2_ID),
            rd_EX.eq(rd_ID),
            opcode_EX.eq(opcode_ID),
            funct3_EX.eq(funct3_ID),
            funct7_EX.eq(funct7_ID),
            imm_EX.eq(imm_ID),
            reg_write_C_EX.eq(reg_write_C_ID),
            mem_to_reg_C_EX.eq(mem_to_reg_C_ID),
            alu_source_C_EX.eq(alu_source_C_ID),
            mem_read_C_EX.eq(mem_read_C_ID),
            mem_write_C_EX.eq(mem_write_C_ID),
            b_type_EX.eq(b_type_ID),
            j_type_EX.eq(j_type_ID),
            u_type_EX.eq(u_type_ID),
            JALR_instr_EX.eq(JALR_instr_ID),
        ]

        # Register file output is already registered,
        # so just assign to rx_EX combinatorially.
        m.d.comb += [
            r1_EX.eq(rp1.data),
            r2_EX.eq(rp2.data),
        ]

        # Fowarding unit signals
        m.d.comb += [
            fwd.i_rs1_EX.eq(rs1_EX),
            fwd.i_rs2_EX.eq(rs2_EX),
            fwd.i_rd_MEM.eq(rd_MEM),
            fwd.i_rd_WB.eq(rd_WB),
            fwd.i_reg_wr_MEM.eq(reg_write_C_MEM),
            fwd.i_reg_wr_WB.eq(reg_write_C_WB),
        ]

        with m.Switch(fwd.o_fwdA):
            with m.Case(0b00):
                m.d.comb += in1_fwd_EX.eq(r1_EX)
            with m.Case(0b01):
                m.d.comb += in1_fwd_EX.eq(data_val_WB)
            with m.Case(0b10):
                m.d.comb += in1_fwd_EX.eq(alu_out_MEM)
            with m.Default():
                m.d.comb += in1_fwd_EX.eq(0)

        with m.Switch(fwd.o_fwdB):
            with m.Case(0b00):
                m.d.comb += in2_fwd_EX.eq(r2_EX)
            with m.Case(0b01):
                m.d.comb += in2_fwd_EX.eq(data_val_WB)
            with m.Case(0b10):
                m.d.comb += in2_fwd_EX.eq(alu_out_MEM)
            with m.Default():
                m.d.comb += in2_fwd_EX.eq(0)

        # in1_EX selection
        with m.If(u_type_EX):
            with m.If(opcode_EX == Opcode.LUI):
                m.d.comb += in1_EX.eq(0)
            with m.Else():
                m.d.comb += in1_EX.eq(pc_EX)
        with m.Elif(j_type_EX):
            m.d.comb += in1_EX.eq(pc_p4_EX)
        with m.Else():
            m.d.comb += in1_EX.eq(in1_fwd_EX)

        # in2_EX selection
        with m.If(j_type_EX):
            m.d.comb += in2_EX.eq(0)
        with m.Else():
            m.d.comb += in2_EX.eq(Mux(alu_source_C_EX, imm_EX, in2_fwd_EX))

        # ALU signals
        m.d.comb += [
            alu.i_in1.eq(in1_EX),
            alu.i_in2.eq(in2_EX),
            alu.i_funct3.eq(Mux(j_type_EX, Funct3.ADD, funct3_EX)),
            alu.i_funct7.eq(Mux(u_type_EX, 0, funct7_EX)),
            alu_out_EX.eq(alu.o_out),
        ]

        # Branch unit signals
        m.d.comb += [
            branch_addr_t_EX.eq(Mux(JALR_instr_EX, r1_EX, pc_EX) + imm_EX),
            branch_addr_EX[1:].eq(branch_addr_t_EX[1:]),
            branch_addr_EX[0].eq(0),
            branch.in1.eq(in1_EX),
            branch.in2.eq(in2_EX),
            branch.br_insn.eq(funct3_EX),
        ]

        #
        # Memory stage (MEM)
        #
        m.d.sync += [
            r2_MEM.eq(r2_EX),
            rd_MEM.eq(rd_EX),
            branch_addr_MEM.eq(branch_addr_EX),
            alu_out_MEM.eq(alu_out_EX),
            reg_write_C_MEM.eq(reg_write_C_EX),
            mem_to_reg_C_MEM.eq(mem_to_reg_C_EX),
            b_type_MEM.eq(b_type_EX),
            j_type_MEM.eq(j_type_EX),
            take_branch_MEM.eq(branch.take_branch),
            mem_read_C_MEM.eq(mem_read_C_EX),
            mem_write_C_MEM.eq(mem_write_C_EX),
        ]

        m.d.comb += self.o_d_addr.eq(alu_out_MEM)
        m.d.comb += self.o_d_data.eq(r2_MEM)
        m.d.comb += data_MEM.eq(self.i_data)
        m.d.comb += self.o_d_Rd.eq(mem_read_C_MEM)
        m.d.comb += self.o_d_Wr.eq(mem_write_C_MEM)

        m.d.comb += valid_branch_MEM.eq(b_type_MEM & take_branch_MEM)
        m.d.comb += valid_jump_MEM.eq(j_type_MEM)
        m.d.comb += pc_source_C_MEM.eq(valid_branch_MEM | valid_jump_MEM)

        #
        # Write-back stage (WB)
        #
        m.d.sync += [
            rd_WB.eq(rd_MEM),
            data_WB.eq(data_MEM),
            alu_out_WB.eq(alu_out_MEM),
            reg_write_C_WB.eq(reg_write_C_MEM),
            mem_to_reg_C_WB.eq(mem_to_reg_C_MEM),
        ]

        m.d.comb += data_val_WB.eq(Mux(mem_to_reg_C_WB, data_WB, alu_out_WB))

        #
        # Connections to the ports of the CPU
        #
        m.submodules.out = out = regfile.read_port()

        m.d.comb += [
            self.o_trap.eq(decoder.trap),
            out.addr.eq(2),
            self.o_reg.eq(out.data),
        ]

        #
        # Formal verification
        #
        if self.formal:
            # Setup
            f_rst_sig = Signal()
            m.d.comb += f_rst_sig.eq(ResetSignal())

            # Check that the LSB of the program counter
            # is always zero. This is required for induction
            # proofs to force it to be zero for n-1 cycles.
            m.d.comb += Assert(pc_IF[0] == 0)

            # Check that we never write to register 0
            with m.If(reg_write_C_WB):
                m.d.comb += Assert(rd_WB != 0)

            # Check that we always assert the trap signal
            # in the next clock cycle whenever we encounter
            # an instruction that we cannot decode properly.
            with m.If((opcode_ID != Opcode.JAL) &
                      (opcode_ID != Opcode.JALR) &
                      (opcode_ID != Opcode.BRANCH) &
                      (opcode_ID != Opcode.LOAD) &
                      (opcode_ID != Opcode.LUI) &
                      (opcode_ID != Opcode.AUIPC) &
                      (opcode_ID != Opcode.OP) &
                      (opcode_ID != Opcode.OP_IMM) &
                      (opcode_ID != Opcode.STORE)):
                m.d.comb += Assert(self.o_trap)
            
            # Check if the JAL instruction jumps to the
            # correct address, and stores the address of
            # the next instruction in the destination
            # register (unless it is x0).
            f_jal_instr = Signal()
            m.d.comb += f_jal_instr.eq(opcode_ID == Opcode.JAL)

            with m.If(Past(f_jal_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                m.d.comb += Assert(pc_IF == Past(branch_addr_MEM))
                # We have to make sure we truncate the result of the addition
                # or else we will be comparing a 32-bit and 33-bit value, which
                # can of course fail.
                m.d.comb += Assert(Past(branch_addr_MEM) == ((Past(pc_ID, 3) + Past(imm_ID, 3))[:32]))
                m.d.comb += Assert(rd_WB == Past(rd_ID, 3))

                with m.If(rd_WB != 0):
                    m.d.comb += Assert(reg_write_C_WB)
                    # Since the CPU is pipelined, pc_IF is always 4 ahead
                    # of the instruction during the ID stage, so don't add
                    # 4 to the Past(pc_IF, 3) statement since it's already
                    # 4 higher.
                    m.d.comb += Assert(data_val_WB == (Past(pc_IF, 3) + 0))
                    m.d.comb += Assert(data_val_WB[0] == 0)
                with m.Else():
                    m.d.comb += Assert(reg_write_C_WB == 0)


            # Check if the JALR instruction jumps to the
            # correct address, and stores the address of
            # the next instruction in the destination
            # register (unless it is x0).
            f_jalr_instr = Signal()
            m.d.comb += f_jalr_instr.eq(opcode_ID == Opcode.JALR)

            with m.If(Past(f_jalr_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                m.d.comb += Assert(pc_IF == Past(branch_addr_MEM))
                # We have to make sure we truncate the result of the addition
                # or else we will be comparing a 32-bit and 33-bit value, which
                # can of course fail. Also, the LSB of the addition should be 0.
                m.d.comb += Assert(Past(branch_addr_MEM) == 
                                   Cat(0b0, ((Past(r1_EX, 2) + Past(imm_ID, 3))[1:32])))
                m.d.comb += Assert(rd_WB == Past(rd_ID, 3))

                with m.If(rd_WB != 0):
                    m.d.comb += Assert(reg_write_C_WB)
                    # Since the CPU is pipelined, pc_IF is always 4 ahead
                    # of the instruction during the ID stage, so don't add
                    # 4 to the Past(pc_IF, 3) statement since it's already
                    # 4 higher.
                    m.d.comb += Assert(data_val_WB == (Past(pc_IF, 3) + 0))
                    m.d.comb += Assert(data_val_WB[0] == 0)
                with m.Else():
                    m.d.comb += Assert(reg_write_C_WB == 0)

            # Check if the LUI instruction loads the U-type
            # immediate in the upper 20 bits of the destination
            # register and sets the lower 12 bits to zero.
            f_lui_instr = Signal()
            m.d.comb += f_lui_instr.eq(opcode_ID == Opcode.LUI)

            with m.If(Past(f_lui_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                with m.If(rd_WB != 0):
                    m.d.comb += Assert(reg_write_C_WB)
                    m.d.comb += Assert(data_val_WB[12:] == (Past(imm_ID, 3)[12:]))
                    m.d.comb += Assert(data_val_WB[:12] == 0)

        return m
 

if __name__ == "__main__":
    formal = True

    top = Module()
    sync = ClockDomain()
    top.domains += sync
    top.submodules.cpu = cpu = Misato(xlen=XLEN.RV32, with_RVFI=False, formal=formal)

    if not formal:
        # Knightrider program
        data = [
            # Setup
            RV32_I(imm= 0x01, rs1=0,        rd=2, funct3=Funct3.ADD),
            RV32_I(imm= 0x01, rs1=0,        rd=3, funct3=Funct3.ADD),
            RV32_I(imm= 0x80, rs1=0,        rd=5, funct3=Funct3.ADD),

            # Left setup
            RV32_I(imm= 0x00, rs1=0,        rd=1, funct3=Funct3.ADD),
            RV32_U(imm= 0x50,               rd=6, opcode=U_Instr.LUI),

            # Left
            RV32_I(imm= 0x01, rs1=1,        rd=1, funct3=Funct3.ADD),
            RV32_B(imm=-0x04, rs1=1, rs2=6,       funct3=Funct3.BLT),
            RV32_I(imm= 0x01, rs1=2,        rd=2, funct3=Funct3.SLL),
            RV32_B(imm= 0x08, rs1=2, rs2=5,       funct3=Funct3.BEQ),
            RV32_J(imm=-0x18,               rd=0, opcode=J_Instr.JAL),

            # Right setup
            RV32_I(imm= 0x00, rs1=0,        rd=1, funct3=Funct3.ADD),
            RV32_U(imm= 0x50,               rd=6, opcode=U_Instr.LUI),

            # Right
            RV32_I(imm= 0x01, rs1=1,        rd=1, funct3=Funct3.ADD),
            RV32_B(imm=-0x04, rs1=1, rs2=6,       funct3=Funct3.BLT),
            RV32_I(imm= 0x01, rs1=2,        rd=2, funct3=Funct3.SR),
            RV32_B(imm=-0x30, rs1=2, rs2=3,       funct3=Funct3.BEQ),
            RV32_J(imm=-0x18,               rd=0, opcode=J_Instr.JAL),
        ]

        imem = Memory(width=32, depth=64, init=data)
        top.submodules.imem_r = imem_r = imem.read_port()

        dmem = Memory(width=32, depth=16)
        top.submodules.dmem_r = dmem_r = dmem.read_port()
        top.submodules.dmem_w = dmem_w = dmem.write_port()

        top.d.comb += imem_r.addr.eq(cpu.o_i_addr[2:])
        # top.d.sync += imem_r.en.eq(cpu.o_i_en)
        top.d.comb += cpu.i_instr.eq(imem_r.data)

        top.d.comb += dmem_r.addr.eq(cpu.o_d_addr)
        # top.d.comb += dmem_r.en.eq(cpu.o_d_Rd)
        top.d.comb += cpu.i_data.eq(dmem_r.data)
        top.d.comb += dmem_w.addr.eq(cpu.o_d_addr)
        top.d.comb += dmem_w.en.eq(cpu.o_d_Wr)
        top.d.comb += dmem_w.data.eq(cpu.o_d_data)

        def bench():
            yield sync.rst.eq(1)
            yield
            yield sync.rst.eq(0)
            yield
            assert not (yield cpu.o_trap)
            
            for _ in range(50):
                yield
                assert not (yield cpu.o_trap)
            
        sim = Simulator(top)
        sim.add_clock(1e-6)
        sim.add_sync_process(bench)
        with sim.write_vcd("cpu.vcd"):
            sim.run()

    parser = main_parser()
    args = parser.parse_args()
    main_runner(parser, args, top, ports=cpu.ports() + [sync.clk, sync.rst])

    # with open("cpu.v", "w") as file:
    #     file.write(verilog.convert(cpu, ports=cpu.ports()))