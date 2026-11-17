! SH-2 kernel for sh2z_vlc8_decode (include/sh2zam/vlc.h). MIT; Copyright (c) 2026 SH2ZAM Contributors. Original code.
! void sh2z_vlc8_decode_sh2(const sh2z_vlc8_t *t /*r4*/, sh2z_bitr_t *r /*r5*/, uint8_t *dst /*r6*/, size_t n /*r7*/)
! Same result as the C reference. r5->p must be 2 byte aligned (mov.w). Preserves r8 to r14 and pr; r14 is never touched (it is -ffixed in some Saturn builds).
! Registers in the loop: r1 bb (marker bit buffer), r2 symbol, r4 0x10000, r5 dst, r7 count, r8 s[], r9 n[], r10 F table, r11 stream pointer, r12 first level mask, r13 F offset of the first level window.
! No variable shift on the SH-2: bb >> len is the high word of dmulu.l bb, 2^(32 - len); F[k] = 2^(31 - k) shifts right by k + 1, indexed by n = 4 * (len - 1).
	.text
	.align 2
	.global _sh2z_vlc8_decode_sh2
_sh2z_vlc8_decode_sh2:
	tst	r7,r7
	bf	1f
	rts
	nop
1:	mov.l	r8,@-r15
	mov.l	r9,@-r15
	mov.l	r10,@-r15
	mov.l	r11,@-r15
	mov.l	r12,@-r15
	mov.l	r13,@-r15
	sts.l	pr,@-r15
	add	#-8,r15
	mov.l	r5,@(0,r15)             ! the reader
	mov	r4,r3
	add	#127,r3
	add	#127,r3
	add	#127,r3
	add	#127,r3
	add	#8,r3                   ! t + 516 = pool
	mov.l	r3,@(4,r15)
	mov	r4,r8
	add	#4,r8                   ! s[]
	mov	r4,r9
	add	#127,r9
	add	#127,r9
	add	#6,r9                   ! n[] = t + 260
	mov.b	@r4,r0
	extu.b	r0,r0
	shll2	r0                      ! 4 * w
	mov	r0,r13
	add	#-4,r13                 ! F offset of the window: 4 * (w - 1)
	mov.l	.LMK,r3
	mov.l	@(r0,r3),r12            ! mask (w) = 2^w - 1
	mov.l	.LF,r10
	mov.l	.L10000,r4
	mov.l	@r5,r11
	mov.l	@(4,r5),r1
	mov	r6,r5                   ! dst
loop:
	mov	r1,r0
	and	r12,r0
	mov.b	@(r0,r8),r2
	mov.b	@(r0,r9),r0
	cmp/pz	r0
	bf	esc
cont:
	mov.l	@(r0,r10),r6
	mov.b	r2,@r5
	add	#1,r5
	dmulu.l	r1,r6
	sts	mach,r1
	cmp/hs	r4,r1
	bf	rf
rfb:
	dt	r7
	bf	loop
	mov.l	@(0,r15),r3
	mov.l	r11,@r3
	mov.l	r1,@(4,r3)
	add	#8,r15
	lds.l	@r15+,pr
	mov.l	@r15+,r13
	mov.l	@r15+,r12
	mov.l	@r15+,r11
	mov.l	@r15+,r10
	mov.l	@r15+,r9
	rts
	mov.l	@r15+,r8

! second level: r0 = n (bit 7 set), r2 = s[e]; returns r2 = symbol, r0 = 4 * (length - 1), clobbers r3, r6
esc:
	mov	r0,r3
	shlr2	r0
	shlr2	r0
	and	#7,r0                   ! xb
	shll2	r0
	mov.l	.LMK,r6
	mov.l	@(r0,r6),r6             ! mask (xb)
	mov	r3,r0
	and	#15,r0
	shll8	r0
	extu.b	r2,r2
	or	r2,r0
	shll	r0                      ! byte offset into the pool
	mov.l	@(4,r15),r3
	add	r3,r0
	mov	r0,r2                   ! &pool[off]
	mov	r13,r0
	mov.l	@(r0,r10),r3
	dmulu.l	r1,r3
	sts	mach,r3                 ! bb >> w
	and	r3,r6
	shll	r6
	add	r6,r2
	mov.w	@r2,r0
	extu.w	r0,r0
	mov	r0,r2
	shlr8	r2
	bra	cont
	extu.b	r0,r0

rf:
	bsr	REFILL
	nop
	bra	rfb
	nop

! REFILL: r1 = bb with fewer than 16 valid bits (bb < 0x10000, marker at bit nb <= 15), r11 = stream pointer: bb += 2^nb * (d + 65535) with d the next 16 bit unit (low byte first). Clobbers r0, r3, r6.
REFILL:
	mov.l	.LHs,r6
	mov	r1,r0
	shlr8	r0
	tst	r0,r0
	bt	2f
	shll	r0
	mov.w	@(r0,r6),r3
	bra	3f
	shll8	r3
2:	mov	r1,r0
	shll	r0
	mov.w	@(r0,r6),r3
3:	mov.w	@r11+,r0
	swap.b	r0,r0
	extu.w	r0,r0
	mul.l	r3,r0
	sts	macl,r0
	add	r0,r1
	sub	r3,r1
	shll16	r3
	rts
	add	r3,r1

	.align 2
.L10000:	.long	0x10000
.LF:	.long	Ftab
.LMK:	.long	MKtab
.LHs:	.long	Hstab
MKtab:	.long	0,1,3,7,15,31,63,127,255
! F[k] = 2^(31 - k) at byte offset 4 * k
Ftab:
	.set n, 0
	.rept 16
	.long 1 << (31 - n)
	.set n, n+1
	.endr
! Hstab[b] = the largest power of two <= b (b = bb for bb < 0x10000, indexed by the marker position): 2^nb for the high byte or low byte, see REFILL
	.align 1
Hstab:
	.short 0,1,2,2,4,4,4,4,8,8,8,8,8,8,8,8
	.rept 16
	.short 16
	.endr
	.rept 32
	.short 32
	.endr
	.rept 64
	.short 64
	.endr
	.rept 128
	.short 128
	.endr
	.align 2
