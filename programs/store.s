    li ra, 0x80
    li gp, 0xAA
    sw gp, 0(ra)

loop:
    j [loop]
