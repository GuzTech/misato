setup:
    li sp, 1
    li gp, 1
    li t0, 0x80

left_setup:
    li ra, 0
    lui t1, 80
left:
    addi ra, ra, 1
    blt ra, t1, [left]

    slli sp, sp, 1
    sw sp, 128(zero)
    beq sp, t0, [right_setup]
    j [left_setup]

right_setup:
    li ra, 0
    lui t1, 80
right:
    add ra, ra, 1
    blt ra, t1, [right]

    srli sp, sp, 1
    sw sp, 128(zero)
    beq sp, gp, [left_setup]
    j [right_setup]
