start:
    addi sp, ra, -1
    addi sp, sp, 2
    addi ra, ra, 2
    addi sp, sp, 2
    jal ra, [start]
