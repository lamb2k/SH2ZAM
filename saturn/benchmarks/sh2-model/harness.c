/* SPDX-License-Identifier: GPL-2.0-or-later
 * Isolated SH-2 execution using Yabause's unmodified GPL interpreter.
 * Copyright (c) 2026 SH2ZAM Contributors. This optional model harness is
 * licensed separately from SH2ZAM's MIT math core; see this directory's LICENSE.
 * Flat RAM excludes caches, wait states, pipeline contention, DMA/peripherals. */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "sh2int.c"

#define RAM_BASE UINT32_C(0x06000000)
#define RAM_SIZE UINT32_C(0x00100000)
#define RETURN_PC UINT32_C(0x06000000)
static unsigned char ram[RAM_SIZE];
static SH2_struct cpu;
static uint64_t instructions,reads,writes,mac_ops,dmuls_ops;
static uint32_t histogram[65536];
yabsys_struct yabsys;

static void fail(const char *why,u32 addr) {
 fprintf(stderr,"%s @%08x pc=%08x instruction=%04x cycles=%u\n",why,addr,cpu.regs.PC,cpu.instruction,cpu.cycles);exit(2);
}
static size_t offset(u32 addr,size_t len) {
 if(len>RAM_SIZE||addr<RAM_BASE||addr-RAM_BASE>RAM_SIZE-len)fail("RAM range",addr);
 return addr-RAM_BASE;
}
static u8 rb(SH2_struct *sh,u32 addr) {(void)sh;reads++;return ram[offset(addr,1)];}
static u16 rw(SH2_struct *sh,u32 addr) {(void)sh;reads++;size_t i=offset(addr,2);return (u16)((u16)ram[i]<<8|ram[i+1]);}
static u32 rl(SH2_struct *sh,u32 addr) {(void)sh;reads++;size_t i=offset(addr,4);return (u32)ram[i]<<24|(u32)ram[i+1]<<16|(u32)ram[i+2]<<8|ram[i+3];}
static void wb(SH2_struct *sh,u32 addr,u8 v) {(void)sh;writes++;ram[offset(addr,1)]=v;}
static void ww(SH2_struct *sh,u32 addr,u16 v) {(void)sh;writes++;size_t i=offset(addr,2);ram[i]=(u8)(v>>8);ram[i+1]=(u8)v;}
static void wl(SH2_struct *sh,u32 addr,u32 v) {(void)sh;writes++;size_t i=offset(addr,4);ram[i]=(u8)(v>>24);ram[i+1]=(u8)(v>>16);ram[i+2]=(u8)(v>>8);ram[i+3]=(u8)v;}
static u32 fetch(SH2_struct *sh,u32 addr) {(void)sh;size_t i=offset(addr,2);u16 op=(u16)((u16)ram[i]<<8|ram[i+1]);instructions++;histogram[op]++;if((op&0xf00f)==0x000f)mac_ops++;if((op&0xf00f)==0x300d)dmuls_ops++;return op;}
/* These entry points are unreachable for the configured pure SH-2 CPU/RAM
 * run. Trap if a supposedly isolated kernel reaches unsupported hardware. */
int BiosHandleFunc(SH2_struct *sh) {(void)sh;fail("Unexpected BIOS",0);return 0;}
void YabSetError(int type,const void *extra) {(void)extra;fail("Interpreter error",(u32)type);}
u16 FASTCALL DataArrayReadWord(SH2_struct *sh,u32 addr) {(void)sh;fail("Unexpected cache array",addr);return 0;}
int sh2_a_bus_check_wait(u32 addr,int size) {(void)size;fail("Unexpected peripheral",addr);return 0;}

struct symbol {char name[128];u32 address;};
static struct symbol symbols[2048];static size_t nsymbols;
static u32 sym(const char *name) {for(size_t i=0;i<nsymbols;i++)if(!strcmp(symbols[i].name,name))return symbols[i].address;fprintf(stderr,"No symbol %s\n",name);exit(2);}
static void load_symbols(const char *path) {FILE*f=fopen(path,"r");if(!f){perror(path);exit(2);}char line[512],type,name[128];unsigned addr;while(fgets(line,sizeof(line),f))if(sscanf(line,"%x %c %127s",&addr,&type,name)==3){if(nsymbols>=2048)exit(2);symbols[nsymbols].address=addr;strcpy(symbols[nsymbols++].name,name);}fclose(f);}
static u16 be16(const unsigned char*p) {return (u16)((u16)p[0]<<8|p[1]);}
static u32 be32(const unsigned char*p) {return (u32)p[0]<<24|(u32)p[1]<<16|(u32)p[2]<<8|p[3];}
static void load_elf(const char *path) {
 FILE*f=fopen(path,"rb");if(!f){perror(path);exit(2);}
 if(fseek(f,0,SEEK_END))fail("Cannot seek ELF",0);
 long length=ftell(f);if(length<52)fail("Truncated ELF header",0);
 rewind(f);unsigned char*b=malloc((size_t)length);if(!b||fread(b,1,(size_t)length,f)!=(size_t)length)exit(2);fclose(f);
 if(memcmp(b,"\177ELF",4)||b[4]!=1||b[5]!=2||be16(b+18)!=42)fail("Expected ELF32 big-endian SuperH",0);
 size_t pos=be32(b+28);u16 stride=be16(b+42),count=be16(b+44);
 if(stride<32||(uint64_t)pos+(uint64_t)stride*count>(uint64_t)length)fail("Invalid ELF program headers",(u32)pos);
 for(unsigned i=0;i<count;i++,pos+=stride){const unsigned char*p=b+pos;if(be32(p)!=1)continue;u32 fileoff=be32(p+4),addr=be32(p+8),filesize=be32(p+16),memsize=be32(p+20);if(addr>=0x26000000)addr-=0x20000000;if(filesize>memsize||(uint64_t)fileoff+filesize>(uint64_t)length)fail("Invalid ELF segment",addr);size_t at=offset(addr,memsize);memset(ram+at,0,memsize);memcpy(ram+at,b+fileoff,filesize);}
 free(b);
}
struct cost {u32 cycles,result,sr;uint64_t instructions,reads,writes,mac_ops,dmuls_ops;};
static struct cost invoke_sr(const char*name,u32 a,u32 b,u32 c,u32 d,u32 sr) {
 static const u32 saved[7]={UINT32_C(0x18293a4b),UINT32_C(0x293a4b5c),UINT32_C(0x3a4b5c6d),UINT32_C(0x4b5c6d7e),UINT32_C(0x5c6d7e8f),UINT32_C(0x6d7e8f90),UINT32_C(0x7e8f90a1)};
 memset(&cpu.regs,0,sizeof(cpu.regs));cpu.regs.R[4]=a;cpu.regs.R[5]=b;cpu.regs.R[6]=c;cpu.regs.R[7]=d;cpu.regs.R[15]=RAM_BASE+RAM_SIZE-64;cpu.regs.SR.all=sr;cpu.regs.PC=sym(name);cpu.regs.PR=RETURN_PC;cpu.cycles=0;cpu.delay=0;cpu.NumberOfInterrupts=0;instructions=reads=writes=mac_ops=dmuls_ops=0;
 for(unsigned r=8;r<15;r++)cpu.regs.R[r]=saved[r-8];
 while(cpu.regs.PC!=RETURN_PC){if(instructions>UINT64_C(100000000))fail("Instruction limit",cpu.regs.PC);cpu.instruction=(u16)fetch(&cpu,cpu.regs.PC);((opcodefunc *)cpu.opcodes)[cpu.instruction](&cpu);}
 if(cpu.regs.R[15]!=RAM_BASE+RAM_SIZE-64)fail("Stack not restored",cpu.regs.R[15]);
 for(unsigned r=8;r<15;r++)if(cpu.regs.R[r]!=saved[r-8])fail("Callee-saved register not restored",r);
 if((cpu.regs.SR.all&2)!=(sr&2))fail("SR.S changed",cpu.regs.SR.all);
 return (struct cost){cpu.cycles,cpu.regs.R[0],cpu.regs.SR.all,instructions,reads,writes,mac_ops,dmuls_ops};
}
static struct cost invoke(const char*name,u32 a,u32 b,u32 c,u32 d) {return invoke_sr(name,a,b,c,d,0);}
static int32_t signed_word(u32 addr) {u32 bits=rl(&cpu,addr);int32_t v;memcpy(&v,&bits,4);return v;}
static int32_t oracle(u32 in,u32 matrix,unsigned row) {
 __int128 sum=0;for(unsigned c=0;c<3;c++)sum+=(__int128)signed_word(in+4*c)*signed_word(matrix+16*row+4*c);
 sum=(sum>=0?sum/65536:-1-((-1-sum)/65536))+signed_word(matrix+16*row+12);
 return sum>INT32_MAX?INT32_MAX:sum<INT32_MIN?INT32_MIN:(int32_t)sum;
}
static const char *kernels[]={"_before_range","_general_range","_after_range","_mac_range","_dmuls_range","_plain_range"};
static const char *names[]={"float","safe","prepared","mac","dmuls","plain"};
static const char *outputs[]={"_float_output","_general_output","_fixed_output","_mac_output","_dmuls_output","_plain_output"};
static void print_cost(struct cost c) {printf("{\"cycles\":%u,\"instructions\":%"PRIu64",\"dataReads\":%"PRIu64",\"dataWrites\":%"PRIu64",\"macInstructions\":%"PRIu64",\"dmulsInstructions\":%"PRIu64"}",c.cycles,c.instructions,c.reads,c.writes,c.mac_ops,c.dmuls_ops);}
#include "../math-model/capture.h"
int main(int argc,char**argv) {
 if(argc<3){fprintf(stderr,"usage: harness benchmark.elf benchmark.sym [maxcases] [maxposes] | --math\n");return 2;}
 int math_mode=argc>3&&!strcmp(argv[3],"--math");
 unsigned case_count=argc>3&&!math_mode?(unsigned)atoi(argv[3]):15,pose_count=argc>4?(unsigned)atoi(argv[4]):21;if(case_count==0||pose_count==0||case_count>15||pose_count>21)return 2;
 load_symbols(argv[2]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 if(math_mode) return capture_math();
 u32 in=sym("_input"),matrix=sym("_fixed_matrix"),cases=sym("_cases"),poses=sym("_poses");
 uint64_t components=0;unsigned mismatches=0;double max_error=0;
 printf("{\"schema\":2,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",\"cases\":[");
 for(unsigned ci=0;ci<case_count;ci++){
  u32 base=cases+16*ci,scene=rl(&cpu,base),detail=rl(&cpu,base+4),n=rl(&cpu,base+8),source=rl(&cpu,base+12);if(n>4096)fail("Case count",n);
  memcpy(ram+offset(in,n*12),ram+offset(source,n*12),n*12);struct cost initial=invoke("_prepare",n,0,0,0);if(initial.result!=1)fail("Input preparation failed",ci);
  if(ci)putchar(',');printf("{\"scene\":\"%s\",\"detail\":%u,\"vertices\":%u,\"meshPreparation\":",(char*)ram+offset(scene,1),detail,n);print_cost(initial);printf(",\"poses\":[");
  for(unsigned pi=0;pi<pose_count;pi++){
   u32 pose=poses+24*pi,ax=rl(&cpu,pose),ay=rl(&cpu,pose+4);invoke("_setup",ax,ay,rl(&cpu,pose+8),rl(&cpu,pose+12));
   struct cost setup=invoke("_plan_prepare",0,0,0,0);if(setup.result!=1)fail("Plan failed",pi);u32 mask=invoke("_candidate_available",0,0,0,0).result;if(mask!=7)fail("Candidate not active",mask);
   struct cost plain_setup=invoke("_plain_prepare",0,0,0,0);if(plain_setup.result!=1||invoke("_plain_available",0,0,0,0).result!=1)fail("Plain fixed baseline not bounded",pi);
   if(pi)putchar(',');printf("{\"ax\":%u,\"ay\":%u,\"availability\":%u,\"plan\":",ax,ay,mask);print_cost(setup);printf(",\"plainAvailable\":true,\"plainPlan\":");print_cost(plain_setup);printf(",\"paths\":{");
   for(unsigned k=0;k<6;k++){struct cost cost=invoke(kernels[k],0,n,0,0);if(k==5&&(cost.result!=1||cost.dmuls_ops!=9u*n||cost.mac_ops))fail("Plain fixed native multiplication audit",pi);if(k)putchar(',');printf("\"%s\":",names[k]);print_cost(cost);}
   printf("},\"checksums\":[");u32 sums[6]={0};double pose_error=0;unsigned bad=0;
   for(unsigned i=0;i<n;i++)for(unsigned r=0;r<3;r++){
    int32_t expected=oracle(in+12*i,matrix,r);for(unsigned k=0;k<6;k++){u32 raw=rl(&cpu,sym(outputs[k])+12*i+4*r);sums[k]=sums[k]*33+raw;if(k){int32_t v;memcpy(&v,&raw,4);if(v!=expected)bad++;}else{float f;memcpy(&f,&raw,4);double err=fabs((double)f-(double)expected/65536);if(!isfinite(err)||err>2.0/65536)fail("Float precision",i);if(err>pose_error)pose_error=err;}}
    components++;
   }
   for(unsigned k=0;k<6;k++){if(k)putchar(',');printf("\"%08x\"",sums[k]);}printf("],\"fixedMismatches\":%u,\"maxFloatError\":%.17g}",bad,pose_error);mismatches+=bad;if(pose_error>max_error)max_error=pose_error;if(bad)fail("Fixed mismatch",pi);
  }
  printf("]}");fflush(stdout);
 }
 printf("],\"checkedComponentsPerFixedPath\":%"PRIu64",\"fixedMismatches\":%u,\"maximumFloatError\":%.17g",components,mismatches,max_error);
 /* Exercise the real native dispatch with S=1 on the final bounded mesh. */
 u32 final_count=rl(&cpu,cases+16*(case_count-1)+8);
 u32 mask_s=invoke_sr("_candidate_available",0,0,0,0,2).result;
 struct cost saturated=invoke_sr("_mac_range",0,final_count,0,0,2);
 unsigned bad_s=0;for(unsigned i=0;i<final_count;i++)for(unsigned r=0;r<3;r++)if(signed_word(sym("_mac_output")+12*i+4*r)!=oracle(in+12*i,matrix,r))bad_s++;
 if(mask_s!=5||saturated.mac_ops||bad_s||(saturated.sr&2)!=2)fail("SR.S fallback",mask_s);
 /* Rebuild input bounds for an adversarial full-range mesh and matrix. */
 const int32_t full_input[9][3]={{INT32_MIN,INT32_MAX,INT32_MAX},{INT32_MAX,INT32_MIN,INT32_MIN},{1,1,1},{-1,-1,1},{65536,-65536,0},{INT32_MIN,INT32_MIN,INT32_MIN},{0,0,0},{INT32_MAX,INT32_MAX,INT32_MAX},{17,-31,91}};
 const int32_t full_matrix[3][4]={{INT32_MAX,INT32_MAX,-INT32_MAX,17},{INT32_MIN,-1,INT32_MAX,-12345},{INT32_MAX,INT32_MIN,INT32_MAX,INT32_MAX}};
 for(unsigned i=0;i<9;i++)for(unsigned r=0;r<3;r++)wl(&cpu,in+12*i+4*r,(u32)full_input[i][r]);
 if(invoke("_prepare",9,0,0,0).result!=1)fail("Full-range preparation",0);
 for(unsigned i=0;i<3;i++)for(unsigned r=0;r<4;r++)wl(&cpu,matrix+16*i+4*r,(u32)full_matrix[i][r]);
 if(invoke("_plan_prepare",0,0,0,0).result!=1)fail("Full-range plan",0);
 u32 full_mask=invoke("_candidate_available",0,0,0,0).result;unsigned full_bad=0;uint64_t full_mac_ops=0;
 for(unsigned k=1;k<5;k++) {struct cost cost=invoke(kernels[k],0,9,0,0);full_mac_ops+=cost.mac_ops;for(unsigned i=0;i<9;i++)for(unsigned r=0;r<3;r++)if(signed_word(sym(outputs[k])+12*i+4*r)!=oracle(in+12*i,matrix,r))full_bad++;}
 if(full_mask||full_bad||full_mac_ops)fail("Full-range fallback",full_mask);
 /* The independent bounded baseline must reject this input without writing;
  * its failure is not relabeled as a library fallback or a measured result. */
 if(invoke("_plain_prepare",0,0,0,0).result||invoke("_plain_available",0,0,0,0).result)fail("Plain full-range proof accepted",0);
 u32 plain_out=sym("_plain_output");for(unsigned i=0;i<27;i++)wl(&cpu,plain_out+4*i,UINT32_C(0x13579bdf));
 struct cost plain_reject=invoke("_plain_range",0,9,0,0);
 if(plain_reject.result||plain_reject.dmuls_ops||plain_reject.mac_ops)fail("Plain full-range execution accepted",0);
 for(unsigned i=0;i<27;i++)if(rl(&cpu,plain_out+4*i)!=UINT32_C(0x13579bdf))fail("Plain rejection modified output",i);
 printf(",\"guards\":{\"saturationMode\":{\"candidateMask\":%u,\"macInstructions\":%"PRIu64",\"fixedMismatches\":%u,\"sourceVertices\":%u,\"srSRetained\":true},\"fullRange\":{\"candidateMask\":%u,\"macInstructions\":%"PRIu64",\"fixedMismatches\":%u,\"sourceVertices\":9,\"plainAvailable\":false,\"plainRejectedWithoutWrites\":true}}}\n",mask_s,saturated.mac_ops,bad_s,final_count,full_mask,full_mac_ops,full_bad);
 return 0;
}
