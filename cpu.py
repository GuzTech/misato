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
        self.xlen     = xlen               # Number of bits
        self.rvfi     = with_RVFI          # Enable RVFI for formal verification
        self.formal   = formal             # Enable formal verification

        # Inputs
        self.i_instr  = Signal(32)         # Fetched instruction (I-mem)
        self.i_data   = Signal(xlen.value) # Fetched data (D-mem)
        self.i_ack    = Signal()           # ACK instruction read request

        # Outputs
        self.o_req    = Signal()           # Request an instruction read
        self.o_i_addr = Signal(xlen.value) # Instruction memory address
        self.o_i_en   = Signal()           # Read enable
        self.o_d_addr = Signal(xlen.value) # Data memory address
        self.o_d_data = Signal(xlen.value) # Data memory value to be written
        self.o_d_Rd   = Signal()           # Read from data memory
        self.o_d_Wr   = Signal()           # Write to data memory
        self.o_trap   = Signal()           # CPU encountered an issue

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
            self.i_req,
            self.i_ack,
            self.o_i_addr,
            self.o_i_en,
            self.o_d_addr,
            self.o_d_data,
            self.o_d_Rd,
            self.o_d_Wr,
            self.o_trap,
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
        addr_F           = Signal(self.xlen.value)  # Instruction memory address
        pc_IF            = Signal(self.xlen.value)  # Current program counter value
        pc_next_IF       = Signal(self.xlen.value)  # Next program counter value
        pc_p4_IF         = Signal(self.xlen.value)  # Current program counter value + 4

        # Branch stall signals
        bubble_count_IF  = Signal(2, reset=1)       # How many NOPs to insert
        insert_bubble_IF = Signal()                 # Insert a NOP instruction
        preload_next_IF  = Signal()                 # When to preload next instruction

        #
        # Instruction decode stage signals (ID)
        #
        pc_ID            = Signal(self.xlen.value)  # Program counter value
        pc_p4_ID         = Signal(self.xlen.value)  # Current program counter value + 4
        instr_ID         = Signal(self.xlen.value)  # Fetched instruction
        stall_ID         = Signal()                 # Stall signal

        # Instruction decoder signals
        format_ID        = Signal(Format)           # Instruction format
        opcode_ID        = Signal(Opcode)           # Instruction opcode
        LOAD_ID          = Signal()                 # LOAD instruction
        STORE_ID         = Signal()                 # STORE instruction
        BRANCH_ID        = Signal()                 # BRANCH instruction
        JALR_ID          = Signal()                 # JALR instruction
        JAL_ID           = Signal()                 # JAL instruction
        OP_IMM_ID        = Signal()                 # OP_IMM instruction
        OP_ID            = Signal()                 # OP instruction
        SYSTEM_ID        = Signal()                 # SYSTEM instruction
        AUIPC_ID         = Signal()                 # AUIPC instruction
        LUI_ID           = Signal()                 # LUI instruction
        jump_ID          = Signal()                 # Jump instruction
        u_instr_ID       = Signal()                 # U-type instruction
        imm_ID           = Signal(self.xlen.value)  # Immediate value
        rs1_ID           = Signal(5)                # Source register #1
        rs2_ID           = Signal(5)                # Source register #2
        rd_ID            = Signal(5)                # Destination register
        funct3_ID        = Signal(Funct3)           # Funct3 field
        funct7_ID        = Signal(Funct7)           # Funct7 field
        trap_ID          = Signal()                 # Illegal instruction
        pc_mod_instr_ID  = Signal()                 # PC modifying instruction

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
        reg_write_C_EX   = Signal()                 # Write to register file
        mem_to_reg_C_EX  = Signal()                 # Mem or ALU result to register
        mem_read_C_EX    = Signal()                 # Read from data memory
        mem_write_C_EX   = Signal()                 # Write to data memory
        alu_source_C_EX  = Signal()                 # ALU input #2 source
        LOAD_EX          = Signal()                 # LOAD instruction
        STORE_EX         = Signal()                 # STORE instruction
        BRANCH_EX        = Signal()                 # BRANCH instruction
        JALR_EX          = Signal()                 # JALR instruction
        OP_IMM_EX        = Signal()                 # OP_IMM instruction
        OP_EX            = Signal()                 # OP instruction
        LUI_EX           = Signal()                 # LUI instruction
        jump_EX          = Signal()                 # Jump instruction
        u_instr_EX       = Signal()                 # U-type instruction
        in1_fwd_EX       = Signal(self.xlen.value)  # EX stage ALU/BR fowarded value #1
        in1_EX           = Signal(self.xlen.value)  # EX stage ALU/BR value #1
        in2_fwd_EX       = Signal(self.xlen.value)  # EX stage ALU/BR fowarded value #2
        in2_EX           = Signal(self.xlen.value)  # EX stage ALU/BR value #2

        #
        #  Memory stage signals (MEM)
        #
        r2_MEM           = Signal(self.xlen.value)  # Source register #2 value
        rd_MEM           = Signal(5)                # Destination register
        funct3_MEM       = Signal(Funct3)           # Funct3 field
        branch_addr_MEM  = Signal(self.xlen.value)  # Branch address
        data_out_MEM     = Signal(self.xlen.value)  # Data value read from data memory
        alu_out_MEM      = Signal(self.xlen.value)  # ALU output
        zero_MEM         = Signal()                 # ALU result is zero
        take_branch_MEM  = Signal()                 # Branch unit output
        pc_source_C_MEM  = Signal()                 # pc_next source selector
        reg_write_C_MEM  = Signal()                 # Write to register file
        mem_to_reg_C_MEM = Signal()                 # Mem or ALU result to register
        mem_read_C_MEM   = Signal()                 # Read from data memory
        mem_write_C_MEM  = Signal()                 # Write to data memory
        LOAD_MEM         = Signal()                 # LOAD instruction
        STORE_MEM        = Signal()                 # STORE instruction
        BRANCH_MEM       = Signal()                 # B type instruction
        jump_MEM         = Signal()                 # J type instruction
        valid_branch_MEM = Signal()                 # Take the branch
        valid_jump_MEM   = Signal()                 # Take the jump

        # 
        # Write-back stage signals (WB)
        #
        rd_WB            = Signal(5)                # Destination register
        data_in_WB       = Signal(self.xlen.value)  # Data value to be written to the register file
        alu_out_WB       = Signal(self.xlen.value)  # ALU output
        data_val_WB      = Signal(self.xlen.value)  # WB stage data value
        reg_write_C_WB   = Signal()                 # Write to register file
        mem_to_reg_C_WB  = Signal()                 # Mem or ALU result to register
        LOAD_WB          = Signal()                 # LOAD instruction
        funct3_WB        = Signal(Funct3)           # Funct3 field

        ##########
        # Stages #
        ##########

        #
        # Instruction fetch stage (IF)
        #
        m.d.comb += pc_p4_IF.eq(pc_IF + 4)
        m.d.comb += pc_next_IF.eq(Mux(pc_source_C_MEM,
                                      branch_addr_MEM,
                                      Mux(pc_mod_instr_ID | preload_next_IF | (~self.i_ack), pc_IF, pc_p4_IF)))

        m.d.sync += pc_IF.eq(pc_next_IF)
        m.d.comb += preload_next_IF.eq(bubble_count_IF > 1)
        m.d.comb += self.o_i_en.eq(~pc_mod_instr_ID)
        m.d.comb += self.o_i_addr.eq(pc_IF)
        m.d.comb += self.o_i_addr.eq(pc_next_IF)

        with m.If(pc_mod_instr_ID):
            m.d.sync += bubble_count_IF.eq(3)
        with m.Elif(insert_bubble_IF):
            m.d.sync += bubble_count_IF.eq(bubble_count_IF - 1)
        m.d.comb += insert_bubble_IF.eq((bubble_count_IF > 0))

        m.d.sync += self.o_req.eq(~(insert_bubble_IF))

        #
        # Instruction decode stage (ID)
        #
        m.d.sync += pc_ID.eq(pc_IF)
        m.d.sync += pc_p4_ID.eq(pc_p4_IF)
        m.d.sync += stall_ID.eq(~self.i_ack)

        m.d.comb += instr_ID.eq(Mux(insert_bubble_IF | (~self.i_ack), NOP, self.i_instr))
        m.d.comb += pc_mod_instr_ID.eq(BRANCH_ID | jump_ID)
        m.d.comb += JALR_ID.eq(opcode_ID == Opcode.JALR)

        m.d.comb += decoder.i_instr.eq(instr_ID)
        m.d.comb += [
            format_ID .eq(decoder.o_format),
            opcode_ID .eq(decoder.o_opcode),
            LOAD_ID   .eq(decoder.o_LOAD),
            STORE_ID  .eq(decoder.o_STORE),
            BRANCH_ID .eq(decoder.o_BRANCH),
            JALR_ID   .eq(decoder.o_JALR),
            JAL_ID    .eq(decoder.o_JAL),
            OP_IMM_ID .eq(decoder.o_OP_IMM),
            OP_ID     .eq(decoder.o_OP),
            SYSTEM_ID .eq(decoder.o_SYSTEM),
            AUIPC_ID  .eq(decoder.o_AUIPC),
            LUI_ID    .eq(decoder.o_LUI),
            jump_ID   .eq(decoder.o_jump),
            imm_ID    .eq(decoder.o_imm),
            rs1_ID    .eq(decoder.o_rs1),
            rs2_ID    .eq(decoder.o_rs2),
            rd_ID     .eq(decoder.o_rd),
            funct3_ID .eq(decoder.o_funct3),
            funct7_ID .eq(decoder.o_funct7),
            u_instr_ID.eq(decoder.o_u_instr),
            trap_ID   .eq(decoder.o_trap)
        ]

        m.d.comb += reg_write_C_ID.eq(
            (LOAD_ID | OP_ID | OP_IMM_ID | u_instr_ID | jump_ID) & (rd_ID != 0))
        m.d.comb += mem_write_C_ID.eq(STORE_ID)
        m.d.comb += mem_read_C_ID.eq(LOAD_ID)
        m.d.comb += mem_to_reg_C_ID.eq(LOAD_ID)
        m.d.comb += alu_source_C_ID.eq(OP_IMM_ID | u_instr_ID | LOAD_ID | STORE_ID)

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
            LOAD_EX.eq(LOAD_ID),
            STORE_EX.eq(STORE_ID),
            BRANCH_EX.eq(BRANCH_ID),
            JALR_EX.eq(JALR_ID),
            OP_IMM_EX.eq(OP_IMM_ID),
            OP_EX.eq(OP_ID),
            LUI_EX.eq(LUI_ID),
            u_instr_EX.eq(u_instr_ID),
            jump_EX.eq(jump_ID),
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
        with m.If(u_instr_EX):
            with m.If(LUI_EX):
                m.d.comb += in1_EX.eq(0)
            with m.Else():
                m.d.comb += in1_EX.eq(pc_EX)
        with m.Elif(jump_EX):
            m.d.comb += in1_EX.eq(pc_p4_EX)
        with m.Else():
            m.d.comb += in1_EX.eq(in1_fwd_EX)

        # in2_EX selection
        with m.If(jump_EX):
            m.d.comb += in2_EX.eq(0)
        with m.Else():
            m.d.comb += in2_EX.eq(Mux(alu_source_C_EX, imm_EX, in2_fwd_EX))

        # ALU signals
        m.d.comb += [
            alu.i_in1.eq(in1_EX),
            alu.i_in2.eq(in2_EX),
            alu.i_funct3.eq(Mux(jump_EX | LOAD_EX | STORE_EX, Funct3.ADD, funct3_EX)),
            alu.i_funct7.eq(funct7_EX),
            alu.i_op_imm.eq(OP_IMM_EX),
            alu.i_op.eq(OP_EX),
            alu_out_EX.eq(alu.o_out),
        ]

        # Branch unit signals
        m.d.comb += [
            branch_addr_t_EX.eq(Mux(JALR_EX, r1_EX, pc_EX) + imm_EX),
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
            data_out_MEM.eq(in2_fwd_EX),
            rd_MEM.eq(rd_EX),
            funct3_MEM.eq(funct3_EX),
            branch_addr_MEM.eq(branch_addr_EX),
            alu_out_MEM.eq(alu_out_EX),
            reg_write_C_MEM.eq(reg_write_C_EX),
            mem_to_reg_C_MEM.eq(mem_to_reg_C_EX),
            LOAD_MEM.eq(LOAD_EX),
            STORE_MEM.eq(STORE_EX),
            BRANCH_MEM.eq(BRANCH_EX),
            jump_MEM.eq(jump_EX),
            take_branch_MEM.eq(branch.take_branch),
            mem_read_C_MEM.eq(mem_read_C_EX),
            mem_write_C_MEM.eq(mem_write_C_EX),
        ]

        m.d.comb += self.o_d_addr.eq(alu_out_MEM)
        # Send word, lower half-word or lowest byte to data memory
        m.d.comb += self.o_d_data[:8].eq(data_out_MEM[:8])
        with m.If(STORE_MEM):
            with m.If(funct3_MEM == Funct3.W):
                m.d.comb += self.o_d_data[8:].eq(data_out_MEM[8:])
            with m.Elif(funct3_MEM == Funct3.H):
                m.d.comb += self.o_d_data[8:].eq(Cat(data_out_MEM[8:16], Repl(0b0, 16)))
            with m.Elif(funct3_MEM == Funct3.B):
                m.d.comb += self.o_d_data[8:].eq(Repl(0b0, 24))

        m.d.comb += self.o_d_Rd.eq(mem_read_C_MEM)
        m.d.comb += self.o_d_Wr.eq(mem_write_C_MEM)

        m.d.comb += valid_branch_MEM.eq(BRANCH_MEM & take_branch_MEM)
        m.d.comb += valid_jump_MEM.eq(jump_MEM)
        m.d.comb += pc_source_C_MEM.eq(valid_branch_MEM | valid_jump_MEM)

        #
        # Write-back stage (WB)
        #
        m.d.sync += [
            rd_WB.eq(rd_MEM),
            alu_out_WB.eq(alu_out_MEM),
            reg_write_C_WB.eq(reg_write_C_MEM),
            mem_to_reg_C_WB.eq(mem_to_reg_C_MEM),
            LOAD_WB.eq(LOAD_MEM),
            funct3_WB.eq(funct3_MEM),
        ]

        m.d.comb += data_in_WB[:8].eq(self.i_data[:8])
        with m.If(LOAD_WB):
            with m.If(funct3_WB == Funct3.W):
                m.d.comb += data_in_WB[8:].eq(self.i_data[8:])
            with m.Elif(funct3_WB == Funct3.H):
                m.d.comb += data_in_WB[8:].eq(Cat(self.i_data[8:16], Repl(self.i_data[15], 16)))
            with m.Elif(funct3_WB == Funct3.HU):
                m.d.comb += data_in_WB[8:].eq(Cat(self.i_data[8:16], Repl(0b0, 16)))
            with m.Elif(funct3_WB == Funct3.B):
                m.d.comb += data_in_WB[8:].eq(Repl(self.i_data[7], 24))
            with m.Elif(funct3_WB == Funct3.BU):
                m.d.comb += data_in_WB[8:].eq(Repl(0b0, 24))

        m.d.comb += data_val_WB.eq(Mux(mem_to_reg_C_WB, data_in_WB, alu_out_WB))

        #
        # Connections to the ports of the CPU
        #
        m.d.comb += [
            self.o_trap.eq(decoder.o_trap),
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
                m.d.comb += Assert(rd_WB == Past(rd_ID, 3))

            # Check that we always assert the trap signal
            # in the next clock cycle whenever we encounter
            # an instruction that we cannot decode properly.
            with m.If((opcode_ID != Opcode.LOAD) &
                      (opcode_ID != Opcode.STORE) &
                      (opcode_ID != Opcode.BRANCH) &
                      (opcode_ID != Opcode.JALR) &
                      (opcode_ID != Opcode.JAL) &
                      (opcode_ID != Opcode.OP_IMM) &
                      (opcode_ID != Opcode.OP) &
                      (opcode_ID != Opcode.SYSTEM) &
                      (opcode_ID != Opcode.AUIPC) &
                      (opcode_ID != Opcode.LUI)):
                m.d.comb += Assert(self.o_trap)

            # Check that if i_stall is high that the program
            # counter stays the same and that bubbles are
            # inserted in the ID stage.
#            with m.If(~self.i_ack):
#                # m.d.comb += Assert(Stable(pc_next_IF))
#                m.d.comb += Assert(Stable(pc_IF))
#                m.d.comb += Assert(instr_ID == NOP)

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
                    # m.d.comb += Assert(data_val_WB == (Past(pc_IF, 3) + 0))

                    # With Wishbone classic, it takes 3 clock cycles to
                    # get a new instruction. Until an ACK is received,
                    # pc_IF remains stable, so add 4.
                    m.d.comb += Assert(data_val_WB == ((Past(pc_IF, 3) + 4)[:32]))
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
                    # m.d.comb += Assert(data_val_WB == (Past(pc_IF, 3) + 0))

                    # With Wishbone classic, it takes 3 clock cycles to
                    # get a new instruction. Until an ACK is received,
                    # pc_IF remains stable, so add 4.
                    m.d.comb += Assert(data_val_WB == ((Past(pc_IF, 3) + 4)[:32]))
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

            # Check if the AUIPC instruction loads the U-type
            # immediate in the upper 20 bits of the destination
            # register and sets the lower 12 bits to zero.
            f_auipc_instr = Signal()
            m.d.comb += f_auipc_instr.eq(opcode_ID == Opcode.AUIPC)

            with m.If(Past(f_auipc_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                m.d.comb += Assert(rd_WB == Past(rd_ID, 3))

                with m.If(rd_WB != 0):
                    m.d.comb += Assert(reg_write_C_WB)
                    m.d.comb += Assert(data_val_WB ==
                                       (Past(pc_ID, 3) +
                                       Cat(Repl(0b0, 12), Past(imm_ID, 3)[12:]))[:32])

            # Check if the SB, SH and SW instructions store the
            # correct data, and calculates the correct address offset.
            f_store_instr = Signal()
            m.d.comb += f_store_instr.eq(opcode_ID == Opcode.STORE)

            # We will use these values to check if a load from 
            # the same address will return the same value.
            f_const_addr   = AnyConst(32)
            f_stored_addr  = Signal(32)
            f_stored_value = Signal(32)
            m.d.comb += f_stored_addr.eq(f_const_addr)
            with m.If(self.o_d_Wr):
                m.d.sync += f_stored_value.eq(self.o_d_data)
            with m.If(self.o_d_Rd & (self.o_d_addr == f_stored_addr)):
                m.d.sync += self.i_data.eq(f_stored_value)

            with m.If(Past(f_store_instr, 2)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      ):
                m.d.comb += Assert(mem_write_C_MEM)
                m.d.comb += Assert(~mem_read_C_MEM)

                # Check if the address calculation is correct
                m.d.comb += Assert(alu_out_MEM == (Past(in1_EX) + Past(imm_EX))[:32])
                m.d.comb += Assert(self.o_d_addr == alu_out_MEM)

                # Check if the correct source register is used. The source
                # register is always in2_fwd_EX because STORE instructions
                # have three inputs (imm, r1 and r2), and the ALU is
                # calculating d_addr = r1 + imm, and only in2_fwd_EX has
                # the correct value of r2.

                # If the instruction was SW
                with m.If(Past(funct3_ID, 2) == Funct3.W):
                    m.d.comb += Assert(self.o_d_data == Past(in2_fwd_EX))
                # If the instruction was SH
                with m.Elif(Past(funct3_ID, 2) == Funct3.H):
                    m.d.comb += Assert(self.o_d_data == Cat(Past(in2_fwd_EX)[:16], Repl(0b0, 16)))
                # If the instruction was SB
                with m.Elif(Past(funct3_ID, 2) == Funct3.B):
                    m.d.comb += Assert(self.o_d_data == Cat(Past(in2_fwd_EX)[:8], Repl(0, 24)))

            # Check if the SB, SH and SW instructions store the
            # correct data, and calculates the correct address offset.
            f_load_instr = Signal()
            m.d.comb += f_load_instr.eq(opcode_ID == Opcode.LOAD)

            with m.If(Past(f_load_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                m.d.comb += Assert(~Past(mem_write_C_MEM))
                m.d.comb += Assert(Past(mem_read_C_MEM))
                m.d.comb += Assert(~Past(self.o_d_Wr))
                m.d.comb += Assert(Past(self.o_d_Rd))

                # Check if the address calculation is correct
                m.d.comb += Assert(Past(alu_out_MEM) == (Past(in1_EX, 2) + Past(imm_EX, 2))[:32])
                m.d.comb += Assert(Past(self.o_d_addr) == Past(alu_out_MEM))

                with m.If(Past(self.o_d_addr) == f_stored_addr):
                    m.d.comb += Assert(self.i_data == f_stored_value)
                    # If the instruction was LW
                    with m.If(Past(funct3_ID, 3) == Funct3.W):
                        m.d.comb += Assert(data_in_WB == f_stored_value)
                    # If the instruction was LH
                    with m.Elif(Past(funct3_ID, 3) == Funct3.H):
                        m.d.comb += Assert(data_in_WB ==
                                           Cat(f_stored_value[:16], Repl(f_stored_value[15], 16)))
                    # If the instruction was LHU
                    with m.Elif(Past(funct3_ID, 3) == Funct3.HU):
                        m.d.comb += Assert(data_in_WB ==
                                           Cat(f_stored_value[:16], Repl(0b0, 16)))
                    # If the instruction was LB
                    with m.Elif(Past(funct3_ID, 3) == Funct3.B):
                        m.d.comb += Assert(data_in_WB ==
                                           Cat(f_stored_value[:8], Repl(f_stored_value[7], 24)))
                    # If the instruction was LBU
                    with m.Elif(Past(funct3_ID, 3) == Funct3.BU):
                        m.d.comb += Assert(data_in_WB == Cat(f_stored_value[:8], Repl(0, 24)))

            # Check if the OP-IMM instructions perform the correct
            # ALU operations and store the results in the correct
            # destination register.
            f_op_imm_instr = Signal()
            m.d.comb += f_op_imm_instr.eq(opcode_ID == Opcode.OP_IMM)

            with m.If(Past(f_op_imm_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                with m.If(rd_WB != 0):
                    m.d.comb += Assert(reg_write_C_WB)

                    # Make sure to use in1_EX because a previous instruction
                    # might not have written to rs1, so use the forwarded value.
                    with m.If(Past(funct3_ID, 3) == Funct3.ADD):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) + Past(imm_ID, 3))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SLT):
                        m.d.comb += Assert(data_val_WB == Mux(
                            Past(in1_EX, 2).as_signed() < Past(imm_ID, 3).as_signed(),
                            Cat(0b1, Repl(0b0, 31)),
                            Repl(0b0, 32)))
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SLTU):
                        m.d.comb += Assert(data_val_WB == Mux(
                            Past(in1_EX, 2) < Past(imm_ID, 3),
                            Cat(0b1, Repl(0b0, 31)),
                            Repl(0b0, 32)))
                    with m.Elif(Past(funct3_ID, 3) == Funct3.XOR):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) ^ Past(imm_ID, 3))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.OR):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) | Past(imm_ID, 3))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.AND):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) & Past(imm_ID, 3))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SLL):
                        m.d.comb += Assert(data_val_WB ==
                                           (Past(in1_EX, 2) << Past(imm_ID, 3)[:5])[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SR):
                        with m.If(Past(funct7_ID, 3) == Funct7.SRL):
                            m.d.comb += Assert(data_val_WB ==
                                               (Past(in1_EX, 2) >> Past(imm_ID, 3)[:5])[:32])
                        with m.Elif(Past(funct7_ID, 3) == Funct7.SRA):
                            m.d.comb += Assert(data_val_WB ==
                                               (Past(in1_EX, 2).as_signed() >> Past(imm_ID, 3)[:5])[:32])
                with m.Else():
                    m.d.comb += Assert(~reg_write_C_WB)

            # Check if the OP-IMM instructions perform the correct
            # ALU operations and store the results in the correct
            # destination register.
            f_op_instr = Signal()
            m.d.comb += f_op_instr.eq(opcode_ID == Opcode.OP)

            with m.If(Past(f_op_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                with m.If(rd_WB != 0):
                    m.d.comb += Assert(reg_write_C_WB)

                    # Make sure to use in1_EX because a previous instruction
                    # might not have written to rs1, so use the forwarded value.
                    with m.If(Past(funct3_ID, 3) == Funct3.ADD):
                        with m.If(Past(funct7_ID, 3) == Funct7.ADD):
                            m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) + Past(in2_EX, 2))[:32])
                        with m.Elif(Past(funct7_ID, 3) == Funct7.SUB):
                            m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) - Past(in2_EX, 2))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SLT):
                        m.d.comb += Assert(data_val_WB == Mux(
                            Past(in1_EX, 2).as_signed() < Past(in2_EX, 2).as_signed(),
                            Cat(0b1, Repl(0b0, 31)),
                            Repl(0b0, 32)))
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SLTU):
                        m.d.comb += Assert(data_val_WB == Mux(
                            Past(in1_EX, 2) < Past(in2_EX, 2),
                            Cat(0b1, Repl(0b0, 31)),
                            Repl(0b0, 32)))
                    with m.Elif(Past(funct3_ID, 3) == Funct3.XOR):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) ^ Past(in2_EX, 2))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.OR):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) | Past(in2_EX, 2))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.AND):
                        m.d.comb += Assert(data_val_WB == (Past(in1_EX, 2) & Past(in2_EX, 2))[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SLL):
                        m.d.comb += Assert(data_val_WB ==
                                           (Past(in1_EX, 2) << Past(in2_EX, 2)[:5])[:32])
                    with m.Elif(Past(funct3_ID, 3) == Funct3.SR):
                        with m.If(Past(funct7_ID, 3) == Funct7.SRL):
                            m.d.comb += Assert(data_val_WB ==
                                               (Past(in1_EX, 2) >> Past(in2_EX, 2)[:5])[:32])
                        with m.Elif(Past(funct7_ID, 3) == Funct7.SRA):
                            m.d.comb += Assert(data_val_WB ==
                                               (Past(in1_EX, 2).as_signed() >> Past(in2_EX, 2)[:5])[:32])
                with m.Else():
                    m.d.comb += Assert(~reg_write_C_WB)

            # Check if BRANCH instruction jump to the
            # correct address if the condition is true.
            f_branch_instr = Signal()
            m.d.comb += f_branch_instr.eq(opcode_ID == Opcode.BRANCH)

            with m.If(Past(f_branch_instr, 3)
                      & (~Past(f_rst_sig))
                      & (~Past(f_rst_sig, 2))
                      & (~Past(f_rst_sig, 3))
                      & (~Past(f_rst_sig, 4))
                      ):
                # Check if the address is calculated correctly
                m.d.comb += Assert(Past(branch_addr_MEM) == ((Past(pc_ID, 3) + Past(imm_ID, 3))[:32]))
                # Check if the program counter is set correctly
                # m.d.comb += Assert(pc_IF == Mux(Past(take_branch_MEM),
                #                                 Past(branch_addr_MEM),
                #                                 Past(pc_p4_ID, 3)))

                # m.d.comb += Assert(pc_next_IF == Mux(Past(take_branch_MEM),
                #                                      Past(branch_addr_MEM),
                #                                      Past(pc_p4_ID, 3)))
                # Check if the condition is processed correctly
                with m.Switch(Past(funct3_ID, 3)):
                    with m.Case(Funct3.BEQ):
                        m.d.comb += Assert(Past(take_branch_MEM) ==
                                           (Past(in1_EX, 2) == Past(in2_EX, 2)))
                    with m.Case(Funct3.BNE):
                        m.d.comb += Assert(Past(take_branch_MEM) ==
                                           (Past(in1_EX, 2) != Past(in2_EX, 2)))
                    with m.Case(Funct3.BLT):
                        m.d.comb += Assert(Past(take_branch_MEM) ==
                                           (Past(in1_EX, 2).as_signed() < Past(in2_EX, 2).as_signed()))
                    with m.Case(Funct3.BGE):
                        m.d.comb += Assert(Past(take_branch_MEM) ==
                                           (Past(in1_EX, 2).as_signed() >= Past(in2_EX, 2).as_signed()))
                    with m.Case(Funct3.BLTU):
                        m.d.comb += Assert(Past(take_branch_MEM) ==
                                           (Past(in1_EX, 2) < Past(in2_EX, 2)))
                    with m.Case(Funct3.BGEU):
                        m.d.comb += Assert(Past(take_branch_MEM) ==
                                           (Past(in1_EX, 2) >= Past(in2_EX, 2)))

        return m
 

class MisatoWB(Elaboratable):
    def __init__(self, xlen: XLEN, formal=False):
        # Configuration
        self.xlen   = xlen
        self.formal = formal

        # Bus
        self.ibus = Interface(addr_width=32,
                              data_width=32,
                              features=["stall", "err"])

        # Inputs
        self.i_data   = Signal(xlen.value) # Fetched data (D-mem)

        # Outputs
        self.o_d_addr = Signal(xlen.value) # Data memory address
        self.o_d_data = Signal(xlen.value) # Data memory value to be written
        self.o_d_Rd   = Signal()           # Read from data memory
        self.o_d_Wr   = Signal()           # Write to data memory
        self.o_trap   = Signal()           # CPU encountered an issue

    def ports(self) -> List[Signal]:
        return [
            self.ibus.stb,
            self.ibus.cyc,
            self.ibus.ack,
            self.ibus.adr,
            self.ibus.dat_r,
            self.i_data,
            self.o_d_addr,
            self.o_d_data,
            self.o_d_Rd,
            self.o_d_Wr,
            self.o_trap,
        ]

    def elaborate(self, platform: Platform) -> Module:
        m = Module()
        m.submodules.cpu = cpu = Misato(xlen=self.xlen, formal=self.formal)

        m.d.comb += [
            self.o_d_addr.eq(cpu.o_d_addr),
            self.o_d_data.eq(cpu.o_d_data),
            self.o_d_Rd.eq(cpu.o_d_Rd),
            self.o_d_Wr.eq(cpu.o_d_Wr),
            self.o_trap.eq(cpu.o_trap),
            cpu.i_data.eq(self.i_data),
        ]

        rdata = Signal.like(self.ibus.dat_r)
        with m.If(self.ibus.cyc):
            with m.If(self.ibus.ack | self.ibus.err | (~cpu.o_req)):
                m.d.sync += [
                    self.ibus.cyc.eq(0),
                    self.ibus.stb.eq(0),
                    rdata.eq(self.ibus.dat_r)
                ]
        with m.Elif(cpu.o_req):
            m.d.sync += [
                self.ibus.adr.eq(cpu.o_i_addr),
                self.ibus.cyc.eq(1),
                self.ibus.stb.eq(1),
            ]

        m.d.comb += self.ibus.sel.eq(0b1111)
        m.d.sync += cpu.i_ack.eq(self.ibus.cyc & self.ibus.ack)
        m.d.comb += cpu.i_instr.eq(rdata)

        return m


if __name__ == "__main__":
    formal = True

    top = Module()
    sync = ClockDomain()
    top.domains += sync
    #top.submodules.cpu = cpu = Misato(xlen=XLEN.RV32, with_RVFI=False, formal=formal)
    top.submodules.cpu = cpu = MisatoWB(xlen=XLEN.RV32, formal=formal)

    if not formal:
        # Knightrider program
        data = [
            # Setup
            RV32_I(imm= 0x01, rs1=0,        rd=2, funct3=Funct3.ADD),   # 0x00
            RV32_I(imm= 0x01, rs1=0,        rd=3, funct3=Funct3.ADD),   # 0x04
            RV32_I(imm= 0x80, rs1=0,        rd=5, funct3=Funct3.ADD),   # 0x08

            # Left setup
            RV32_I(imm= 0x00, rs1=0,        rd=1, funct3=Funct3.ADD),   # 0x0C
            #RV32_U(imm= 0x50,               rd=6, opcode=U_Instr.LUI), # 0x10
            RV32_I(imm= 0x00, rs1=0,        rd=6, funct3=Funct3.ADD),   # 0x10

            # Left
            RV32_I(imm= 0x01, rs1=1,        rd=1, funct3=Funct3.ADD),   # 0x14
            RV32_B(imm=-0x04, rs1=1, rs2=6,       funct3=Funct3.BLT),   # 0x18
            RV32_I(imm= 0x01, rs1=2,        rd=2, funct3=Funct3.SLL),   # 0x1C
            RV32_B(imm= 0x08, rs1=2, rs2=5,       funct3=Funct3.BEQ),   # 0x20
            RV32_J(imm=-0x18,               rd=0, opcode=J_Instr.JAL),  # 0x24

            # Right setup
            RV32_I(imm= 0x00, rs1=0,        rd=1, funct3=Funct3.ADD),   # 0x28
            # RV32_U(imm= 0x50,               rd=6, opcode=U_Instr.LUI),  # 0x2C
            RV32_I(imm= 0x00, rs1=0,        rd=6, funct3=Funct3.ADD),   # 0x2C

            # Right
            RV32_I(imm= 0x01, rs1=1,        rd=1, funct3=Funct3.ADD),   # 0x30
            RV32_B(imm=-0x04, rs1=1, rs2=6,       funct3=Funct3.BLT),   # 0x34
            RV32_I(imm= 0x01, rs1=2,        rd=2, funct3=Funct3.SR),    # 0x38
            RV32_B(imm=-0x30, rs1=2, rs2=3,       funct3=Funct3.BEQ),   # 0x3C
            RV32_J(imm=-0x18,               rd=0, opcode=J_Instr.JAL),  # 0x40
        ]

        # data = [
        #     RV32_I(imm=-1, rs1=0,        rd=1, funct3=Funct3.ADD),
        #     RV32_I(imm= 2, rs1=0,        rd=2, funct3=Funct3.ADD),
        #     RV32_I(imm= 3, rs1=0,        rd=3, funct3=Funct3.ADD),
        #     RV32_I(imm= 4, rs1=0,        rd=4, funct3=Funct3.ADD),
        #     RV32_I(imm= 5, rs1=0,        rd=5, funct3=Funct3.ADD),
        #     RV32_I(imm= 6, rs1=0,        rd=6, funct3=Funct3.ADD),
        #     RV32_S(imm= 4, rs1=0, rs2=1,       funct3=Funct3.W),
        #     RV32_J(imm=-8,               rd=0, opcode=Opcode.JAL),
        # ]

        # imem = Memory(width=32, depth=64, init=data)
        # top.submodules.imem_r = imem_r = imem.read_port()
        top.submodules.rom = rom = ROM(data)
        top.d.comb += cpu.ibus.connect(rom.arb.bus)

        dmem = Memory(width=32, depth=256)
        top.submodules.dmem_r = dmem_r = dmem.read_port()
        top.submodules.dmem_w = dmem_w = dmem.write_port()

        # top.d.comb += imem_r.addr.eq(cpu.o_i_addr[2:])
        # top.d.comb += cpu.i_instr.eq(imem_r.data)

        top.d.comb += dmem_r.addr.eq(cpu.o_d_addr)
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

            for _ in range(1000):
                yield
                assert not (yield cpu.o_trap)

        sim = Simulator(top)
        sim.add_clock(1e-6)
        sim.add_sync_process(bench)
        with sim.write_vcd("cpu.vcd"):
            sim.run()

        # with open("cpu.v", "w") as file:
        #     file.write(verilog.convert(cpu, ports=cpu.ports()))
    else:
        parser = main_parser()
        args = parser.parse_args()
        main_runner(parser, args, top, ports=cpu.ports() + [sync.clk, sync.rst])
