start:
    addi ra, zero, 1
    slli ra, ra, 1

loop:
    beq ra, sp, [end]
    addi sp, sp, 1
    j [loop]
    
.word 0, 0, 0, 0

end:
    j [start]
