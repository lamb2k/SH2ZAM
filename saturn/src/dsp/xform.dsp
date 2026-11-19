; SCU DSP kernel for sh2z_dsp_xform (dsp_experimental.h, contract tag dsp-1).
; Assemble with saturn/tools/sh2z_dspasm.py; the words are committed in
; src/sh2zam_dsp_xform.inc and checked against this file in CI.
;
; Copyright (c) 2026 SH2ZAM Contributors. MIT License.
;
; Entry at 0 with M3[1] = D0 word address of the job (16 words: m[3][4],
; in_d0, out_d0, blocks, count). Per block of 20 vectors:
;   DMA 60 input words into M1, three passes (one per output row), DMA the
;   60 results from M2[1..60] out, advance both D0 word addresses by 60.
; Data RAM: M0[0..11] the rows (r0 r1 r2 t), M0[12..15] the job tail;
;   M1[0..59] inputs; M2[0] pipeline fill, M2[1..60] results;
;   M3[0] = 1 (the X operand of the translation), M3[1] in, M3[2] out,
;   M3[3] blocks left.
;
; One pass, five slots per vector (k), loads (x,r0) (y,r1) (z,r2) (1,t):
;   s0  ADD: A += t of k-1 (final)        X <- x       Y <- r0
;   s1  NOP: ALU = A, stored to MC2;      P <- x*r0    X <- y   CLR A, Y <- r1
;   s2  ADD: A = 0 + x*r0                 P <- y*r1    X <- z   Y <- r2
;   s3  ADD: A += y*r1                    P <- z*r2    X <- 1   Y <- t
;   BTM (loop; s4 is its delay slot)
;   s4  ADD: A += z*r2                    P <- 1*t     CT0 <- row base
; The store in s1 reads the ALU on an ALU-NOP instruction, so it sees the
; finished sum whichever ALU result the D1 bus samples. The first store of
; each block's first pass is the pipeline fill (M2[0]); each later pass's
; first store is the previous pass's last result; a flush ends the block.
; ADD is the 32-bit ALU operation and P.L the low 32 bits of the product,
; so every result is the sum modulo 2^32.

        MOV #1,CT3
        MOV M3,RA0              ; job address
        MOV #0,CT0
        DMA D0,MC0,#16          ; M0[0..15] = job
W0:     JMP T0,W0
        NOP
        MOV #0,CT3
        MVI #1,MC3              ; M3[0] = 1, CT3 = 1
        MOV #12,CT0
        MOV MC0,MC3             ; M3[1] = in
        MOV MC0,MC3             ; M3[2] = out
        MOV MC0,MC3             ; M3[3] = blocks

BLOCK:  MOV #1,CT3
        MOV M3,RA0
        MOV #0,CT1
        DMA D0,MC1,#60          ; 20 input vectors
W1:     JMP T0,W1
        NOP
        MOV #0,CT3              ; X operand 1 for the translation
        MOV #0,CT2              ; fill store lands in M2[0]

        MOV #0,CT0              ; pass 0: row 0
        MOV #0,CT1
        MVI #19,LOP
        MOV #P0,TOP
P0:     ADD | MOV MC1,X | MOV ALU,A | MOV MC0,Y
        NOP | MOV MUL,P | MOV MC1,X | CLR A | MOV MC0,Y | MOV ALL,MC2
        ADD | MOV MUL,P | MOV MC1,X | MOV ALU,A | MOV MC0,Y
        ADD | MOV MUL,P | MOV M3,X | MOV ALU,A | MOV MC0,Y
        BTM
        ADD | MOV MUL,P | MOV ALU,A | MOV #0,CT0

        MOV #4,CT0              ; pass 1: row 1
        MOV #0,CT1
        MVI #19,LOP
        MOV #P1,TOP
P1:     ADD | MOV MC1,X | MOV ALU,A | MOV MC0,Y
        NOP | MOV MUL,P | MOV MC1,X | CLR A | MOV MC0,Y | MOV ALL,MC2
        ADD | MOV MUL,P | MOV MC1,X | MOV ALU,A | MOV MC0,Y
        ADD | MOV MUL,P | MOV M3,X | MOV ALU,A | MOV MC0,Y
        BTM
        ADD | MOV MUL,P | MOV ALU,A | MOV #4,CT0

        MOV #8,CT0              ; pass 2: row 2
        MOV #0,CT1
        MVI #19,LOP
        MOV #P2,TOP
P2:     ADD | MOV MC1,X | MOV ALU,A | MOV MC0,Y
        NOP | MOV MUL,P | MOV MC1,X | CLR A | MOV MC0,Y | MOV ALL,MC2
        ADD | MOV MUL,P | MOV MC1,X | MOV ALU,A | MOV MC0,Y
        ADD | MOV MUL,P | MOV M3,X | MOV ALU,A | MOV MC0,Y
        BTM
        ADD | MOV MUL,P | MOV ALU,A | MOV #8,CT0

        ADD | MOV ALU,A         ; flush: the last result
        NOP | MOV ALL,MC2

        MOV #2,CT3
        MOV M3,WA0
        MOV #1,CT2
        DMA MC2,D0,#60          ; 20 results
W2:     JMP T0,W2
        NOP

        MOV #1,CT3              ; in += 60 words, out += 60 words, blocks -= 1
        MOV M3,A | MOV #60,PL
        ADD | MOV ALU,A
        NOP | MOV ALL,MC3
        MOV M3,A
        ADD | MOV ALU,A
        NOP | MOV ALL,MC3
        MOV M3,A | MOV #1,PL
        SUB | MOV ALU,A
        NOP | MOV ALL,MC3
        JMP NZ,BLOCK
        NOP
        END
