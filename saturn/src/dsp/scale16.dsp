; SCU DSP kernel for sh2z_dsp_scale16 (dsp_experimental.h, contract tag dsp-1).
; Assemble with saturn/tools/sh2z_dspasm.py; the words are committed in
; src/sh2zam_dsp_scale16.inc and checked against this file in CI.
;
; Copyright (c) 2026 SH2ZAM Contributors. MIT License.
;
; out[i * m + k] = floor(a[i] * b[k] / 65536), i < n, k < m (1 <= n, m <= 16),
; from the 48-bit product (AD2 onto a cleared A, ALH = bits 47..16).
; Entry at 0 with M3[0] = D0 word address of the job: b_d0, out_d0, n, m,
; a[16] (20 words), then b[16] at b_d0.
; Data RAM: M0[0..3] header, M0[4..19] a; M1[0..15] b; M2[0..m-1] one row;
;   M3[0] out address of the next row, M3[1] rows left, M3[2] m, M3[3] m - 1.
; Per output, four words: P <- a*b (and Y <- next b, A <- 0); A <- 0 + P;
; BTM; store ALH on an ALU-NOP word (the delay slot), so the store sees the
; finished sum whichever ALU result the D1 bus samples.

        MOV #0,CT3
        MOV M3,RA0              ; job
        MOV #0,CT0
        DMA D0,MC0,#20          ; header and a
W0:     JMP T0,W0
        NOP
        MOV #0,CT0
        MOV MC0,RA0             ; b_d0
        MOV #0,CT1
        DMA D0,MC1,#16          ; b
W1:     JMP T0,W1
        NOP
        MOV #0,CT3
        MOV MC0,MC3             ; M3[0] = out
        MOV MC0,MC3             ; M3[1] = n
        MOV MC0,MC3             ; M3[2] = m; CT0 = 4, the first a
        MOV #2,CT3
        MOV M3,A | MOV #1,PL
        SUB | MOV ALU,A
        MOV #3,CT3
        NOP | MOV ALL,MC3       ; M3[3] = m - 1

ROW:    MOV #0,CT1
        MOV #3,CT3
        MOV MC0,X               ; a[i]
        MOV MC1,Y | MOV M3,LOP  ; b[0]; m - 1 more after the first
        MOV #0,CT2
        MOV #INNER,TOP
INNER:  MOV MUL,P | MOV MC1,Y | CLR A
        AD2 | MOV ALU,A
        BTM
        NOP | MOV ALH,MC2

        MOV #0,CT3
        MOV M3,WA0
        MOV #0,CT2
        MOV #2,CT3
        DMA MC2,D0,M3           ; m words
W2:     JMP T0,W2
        NOP
        MOV M3,P                ; P = m (CT3 = 2)
        MOV #0,CT3
        MOV M3,A
        ADD | MOV ALU,A
        NOP | MOV ALL,MC3       ; out += m words
        MOV M3,A | MOV #1,PL
        SUB | MOV ALU,A
        NOP | MOV ALL,MC3       ; rows -= 1
        JMP NZ,ROW
        NOP
        END
