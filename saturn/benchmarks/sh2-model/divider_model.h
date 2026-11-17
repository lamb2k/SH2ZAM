/* SPDX-License-Identifier: GPL-2.0-or-later
 * HARNESS CODE, NOT A HARDWARE DESCRIPTION. Copyright (c) 2026 SH2ZAM Contributors.
 *
 * The pinned Yabause interpreter is CPU-only; on-chip registers are not
 * mapped, so the SH-2 division unit at 0xFFFFFF00 needs a stand-in for code
 * that uses it explicitly. This model implements exactly the assumptions
 * written in saturn/include/sh2zam/divu_experimental.h and nothing else:
 *
 *   - 32-bit accesses to DVSR (0x00), DVDNTH (0x10) and DVDNTL (0x14) only;
 *     any other register, access width or address in the block traps.
 *   - Writing DVDNTL starts a signed 64/32 division of DVDNTH:DVDNTL by DVSR,
 *     truncating toward zero; DVDNTL gets the quotient, DVDNTH the remainder.
 *   - It completes DIVU_LATENCY cycles after the write. Reading DVDNTL or
 *     DVDNTH earlier adds the remaining cycles to the interpreter's counter
 *     (the "read holds the CPU" assumption) and counts them as stall.
 *   - Any division the library promised never to issue traps: a zero divisor,
 *     |n| >= |d| * 2^31, or any write while a division is running.
 *
 * Agreement between this model and SH2ZAM is therefore not evidence about the
 * console; it shows the library honours its own promises and gives a cycle
 * figure under the stated latency assumption. */
#define DIVU_BASE UINT32_C(0xFFFFFF00)
#define DIVU_LATENCY 39u
static u32 divu_dvsr,divu_dvdnth,divu_dvdntl,divu_started;
static int divu_running;
static uint64_t divu_divisions,divu_stall_cycles,divu_accesses;

static int divu_address(u32 addr) {return addr>=DIVU_BASE&&addr<DIVU_BASE+0x20u;}
static void divu_reset(void) {divu_dvsr=divu_dvdnth=divu_dvdntl=0;divu_running=0;divu_divisions=divu_stall_cycles=divu_accesses=0;}
static void divu_settle(SH2_struct *sh) {
 if(!divu_running)return;
 u32 elapsed=sh->cycles-divu_started;
 if(elapsed<DIVU_LATENCY){sh->cycles+=DIVU_LATENCY-elapsed;divu_stall_cycles+=DIVU_LATENCY-elapsed;}
 divu_running=0;
}
static u32 divu_read(SH2_struct *sh,u32 addr) {
 divu_accesses++;
 switch(addr-DIVU_BASE) {
  case 0x00: return divu_dvsr;
  case 0x10: divu_settle(sh);return divu_dvdnth;
  case 0x14: divu_settle(sh);return divu_dvdntl;
  default: fail("Unmodelled divider register read",addr);return 0;
 }
}
static void divu_write(SH2_struct *sh,u32 addr,u32 value) {
 divu_accesses++;
 if(divu_running&&(u32)(sh->cycles-divu_started)<DIVU_LATENCY)fail("Divider write while dividing (unmodelled)",addr);
 divu_running=0;
 switch(addr-DIVU_BASE) {
  case 0x00: divu_dvsr=value;return;
  case 0x10: divu_dvdnth=value;return;
  case 0x14: {
   int64_t n=(int64_t)(((uint64_t)divu_dvdnth<<32)|value);
   int64_t d=(int32_t)divu_dvsr;
   __int128 magnitude=n<0?-(__int128)n:(__int128)n,limit=(__int128)(d<0?-d:d)<<31;
   if(d==0||magnitude>=limit)fail("Division outside the library's promise",addr);
   divu_dvdntl=(u32)(int32_t)(n/d);divu_dvdnth=(u32)(int32_t)(n%d);
   divu_started=sh->cycles;divu_running=1;divu_divisions++;
   return;
  }
  default: fail("Unmodelled divider register write",addr);
 }
}
