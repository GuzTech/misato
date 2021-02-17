loop:
    addi sp, sp, 1
    sw ra, 0(sp)
    addi ra, ra, 4
    j [loop]
