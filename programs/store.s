    li ra, 0x80
    li gp, 0xAA
    sw gp, 128(zero)

loop:
    j [loop]
