    li ra, 0x80
    li sp, 0
start:
    addi sp, sp, 1
    sw sp, 0(ra)
    j [start]
