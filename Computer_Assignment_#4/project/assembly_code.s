    add s0,zero,zero
    lw s1,0(s0)
    addi t1,zero,4
LOOP:
    slti t3,t1,40
    beq t3,zero,END_LOOP
    add s2,s0,t1
    lw t4,0(s2)
    blt s1,t4,ELSE
    addi t1,t1,4
    J LOOP
ELSE:
    addi t1,t1,4
    add s1,zero,t4
    J LOOP
END_LOOP:
