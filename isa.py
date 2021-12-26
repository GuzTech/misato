from enum import Enum, unique

from amaranth import *
from amaranth.build import Platform


# ADDI R0, R0, 0
NOP = 0x13


@unique
class XLEN(Enum):
    RV32 = 32
    RV64 = 64


@unique
class Opcode(Enum):
    LOAD   = 0b00_000_11
    STORE  = 0b01_000_11
    BRANCH = 0b11_000_11
    JALR   = 0b11_001_11
    JAL    = 0b11_011_11
    OP_IMM = 0b00_100_11
    OP     = 0b01_100_11
    SYSTEM = 0b11_100_11
    AUIPC  = 0b00_101_11
    LUI    = 0b01_101_11


@unique
class Format(Enum):
    R_type       = 0
    I_type       = 1
    S_type       = 2
    B_type       = 3
    U_type       = 4
    J_type       = 5
    Unknown_type = 6


class Funct3(Enum):
    ADD = BEQ  = 0b000
    SLL = BNE  = 0b001
    SLT        = 0b010
    SLTU       = 0b011
    XOR = BLT  = 0b100
    SR  = BGE  = 0b101
    OR  = BLTU = 0b110
    AND = BGEU = 0b111


class Funct7(Enum):
    SRL = ADD = 0b0000000
    SRA = SUB = 0b0100000


class U_Instr(Enum):
    LUI   = 0b0110111
    AUIPC = 0b0010111


class U_Type(Enum):
    LUI   = 0
    AUIPC = 1


class J_Instr(Enum):
    JAL  = 0b1101111
    JALR = 0b1100111


@unique
class Instruction(Enum):
    LD = 0
    ST = 1
    LUI = 2
    ADD = 3
    SUB = 4
    AND = 5
    OR = 6
    XOR = 7


def RV32_R(rs1: int, rs2: int, rd: int, funct3: Funct3, funct7: Funct7):
    return (((funct7.value & 0x7F) << 25) |
            ((rs2          & 0x1F) << 20) |
            ((rs1          & 0x1F) << 15) |
            ((funct3.value & 0x07) << 12) |
            ((rd           & 0x1F) <<  7) |
            Opcode.OP.value)


def RV32_I(imm: int, rs1: int, rd: int, funct3: Funct3):
    return (((imm          & 0xFFF) << 20) |
            ((rs1          & 0x01F) << 15) |
            ((funct3.value & 0x007) << 12) |
            ((rd           & 0x01F) <<  7) |
            Opcode.OP_IMM.value)


def RV32_S(imm: int, rs1: int, rs2: int, funct3: Funct3):
    return (((imm          & 0x230) << 20) |
            ((rs2          & 0x01F) << 20) |
            ((rs1          & 0x01F) << 15) |
            ((funct3.value & 0x007) << 12) |
            ((imm          & 0x01F) <<  7) |
            Opcode.STORE.value)


def RV32_U(imm: int, rd: int, opcode: U_Instr):
    return (((imm & 0xFFFFF) << 12) |
            ((rd  & 0x1F)    <<  7) |
            opcode.value)


def RV32_B(imm: int, rs1: int, rs2: int, funct3: Funct3):
    return (((imm          & 0x1000) << 19) |
            ((imm          & 0x07E0) << 20) |
            ((rs2          & 0x001F) << 20) |
            ((rs1          & 0x001F) << 15) |
            ((funct3.value & 0x0007) << 12) |
            ((imm          & 0x001E) <<  7) |
            ((imm          & 0x0800) >>  4) |
            Opcode.BRANCH.value)


def RV32_J(imm: int, rd: int, opcode: J_Instr):
    return ((((imm & 0x100000) >> 20) << 31) |
            (((imm & 0x0007FE) >>  1) << 21) |
            (((imm & 0x000800) >> 11) << 20) |
            (((imm & 0x0FF000) >> 12) << 12) |
            (((rd  & 0x00001F) >>  0) <<  7) |
            opcode.value)