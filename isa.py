from enum import Enum, unique

from nmigen import *
from nmigen.build import Platform


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
    R_type = 0
    I_type = 1
    S_type = 2
    B_type = 3
    U_type = 4
    J_type = 5


class Funct3(Enum):
    BEQ  = ADD = 0b000
    BNE  = SLL = 0b001
    SLT  =       0b010
    SLTU =       0b011
    BLT  = XOR = 0b100
    BGE  = SR  = 0b101
    BLTU = OR  = 0b110
    BGEU = AND = 0b111


class Funct7(Enum):
    SRL = ADD = 0b0000000
    SRA = SUB = 0b0100000


class U_Instr(Enum):
    LUI   = 0
    AUIPC = 1


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
