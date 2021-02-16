setup:
    li gp, 1
    lui sp, 0x100
    lui tp, 0x100
    lui t0, 0x8000

invert:
    xori gp, gp, 1

reset_timer:
    lui ra, 0x0C5
    addi ra, ra, -1216

count_down:
    beqz ra, [count_is_zero]
    addi ra, ra, -1
    j [count_down]

count_is_zero:
    bnez gp, [led_min_value]

led_max_value:
    beq t0, sp, [invert]
    slli sp, sp, 1
    j [reset_timer]

led_min_value:
    beq tp, sp, [invert]
    srli sp, sp, 1
    j [reset_timer]
