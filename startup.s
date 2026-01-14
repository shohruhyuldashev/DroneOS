// startup.s - DroneOS Boot Code
// ARM Cortex-A53 (ARMv8-A 32-bit mode)

.section .text.boot
.global _start
.extern kernel_main

_start:
    // ==== Only Core 0 runs ====
    mrc p15, 0, r0, c0, c0, 5
    ands r0, r0, #3
    bne halt

    // ==== Setup Stack ====
    ldr sp, =_stack_top

    // ==== Clear BSS ====
    ldr r0, =_bss_start
    ldr r1, =_bss_end
    mov r2, #0

bss_loop:
    cmp r0, r1
    strlt r2, [r0], #4
    blt bss_loop

    // ==== Jump to kernel ====
    bl kernel_main

halt:
    wfi
    b halt
