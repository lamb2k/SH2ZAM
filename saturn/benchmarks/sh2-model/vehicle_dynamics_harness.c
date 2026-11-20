/* SPDX-License-Identifier: GPL-2.0-or-later
 * Executed-instruction checks and cycle model for vehicle-dynamics-1
 * (sh2z_vd_step, sh2z_vd_locate and sh2z_vd_lap_distance,
 * vehicle_dynamics_experimental.h). Runs the actual big-endian SH-2 object
 * code in Yabause's unmodified interpreter, like harness.c, whose loader, call
 * wrapper and divider model it reuses. Flat RAM, no caches or wait states:
 * MODELLED cycles, not Saturn timing. Copyright (c) 2026 SH2ZAM Contributors.
 * This optional model harness is licensed separately from SH2ZAM's MIT core;
 * see LICENSE.
 *
 * usage: vehicle_dynamics_harness model.elf [locate-every] > results.json
 * The ELF must contain bridge.o from benchmarks/vehicle-dynamics-model and the
 * library. The contract promises identical bits on the host and the SH-2, so
 * the oracle is the host build of the same source: after every tick of every
 * input script, every word of the SH-2 body must equal the host body, and
 * every locate and lap-distance probe must return the same value and leave
 * the same probe body. Any mismatch exits non-zero before any timing is
 * printed. The track is a synthetic stadium (two straights joined by half
 * circles) with a boost plate, a recharge strip, a mine and a launch ramp. */
#define main sh2_model_main
#include "harness.c"
#undef main
#include "sh2zam/vehicle_dynamics_experimental.h"
#include "sh2zam/unit30_experimental.h"
#include <stddef.h>

static void load_elf_symbols(const char *path) {
 FILE*f=fopen(path,"rb");if(!f){perror(path);exit(2);}
 fseek(f,0,SEEK_END);long length=ftell(f);rewind(f);
 unsigned char*b=malloc((size_t)length);if(!b||fread(b,1,(size_t)length,f)!=(size_t)length)exit(2);fclose(f);
 u32 shoff=be32(b+32);u16 shentsize=be16(b+46),shnum=be16(b+48);
 for(unsigned i=0;i<shnum;i++){
  const unsigned char*sh=b+shoff+(size_t)i*shentsize;
  if(be32(sh+4)!=2)continue; /* SHT_SYMTAB */
  u32 off=be32(sh+16),size=be32(sh+20),link=be32(sh+24);
  const unsigned char*str=b+be32(b+shoff+(size_t)link*shentsize+16);
  for(u32 p=off;p+16<=off+size;p+=16){
   const char*name=(const char*)str+be32(b+p);
   if(!*name||nsymbols>=2048)continue;
   if(strlen(name)>=sizeof symbols[0].name)continue;
   symbols[nsymbols].address=be32(b+p+4);strcpy(symbols[nsymbols++].name,name);
  }
 }
 free(b);
}
static u32 symc(const char *name){char t[140];snprintf(t,sizeof t,"_%s",name);return sym(t);}
static struct cost call_sr(const char *name,u32 a,u32 b,u32 c,u32 sr){char t[140];snprintf(t,sizeof t,"_%s",name);return invoke_sr(t,a,b,c,0,sr);}
static void bad(const char *what,uint64_t it){fprintf(stderr,"vehicle-dynamics model mismatch: %s (tick %"PRIu64")\n",what,it);exit(3);}

#define PI 3.14159265358979323846
static int32_t q16(double x){return (int32_t)lround(x*65536.0);}
static int32_t q30(double x){return (int32_t)lround(x*1073741824.0);}
static int32_t q12(double x){return (int32_t)lround(x*4096.0);}
static double f16(int32_t v){return v/65536.0;}
static double f30(int32_t v){return v/1073741824.0;}

/* ---- the host track, vehicle and rules (as tests/test_vehicle_dynamics.c) ---- */
enum {N=16};
static sh2z_vd_node_t nodes[N];
static sh2z_vd_zone_t zones[2];
static sh2z_vd_ramp_t ramps[1];
static sh2z_vd_mine_t mines[1];
static sh2z_vd_track_t track;
static sh2z_vd_vehicle_t veh;
static sh2z_vd_rules_t rules;

static void point(int i,double p[3]) {
 const double half=6000.0,r=3000.0;
 if(i<4){p[0]=-half+i*3000.0;p[2]=-r;}
 else if(i<8){double a=-PI/2+(i-3)*PI/5;p[0]=half+r*cos(a);p[2]=r*sin(a);}
 else if(i<12){p[0]=half-(i-8)*3000.0;p[2]=r;}
 else {double a=PI/2+(i-11)*PI/5;p[0]=-half+r*cos(a);p[2]=r*sin(a);}
 p[1]=0.0;
}
static void build_track(void) {
 double total=0.0;
 memset(nodes,0,sizeof nodes);
 for(int i=0;i<N;i++){
  double p[3];point(i,p);
  for(int k=0;k<3;k++)nodes[i].pos[k]=q16(p[k]);
  nodes[i].up[1]=q30(1.0);nodes[i].tension=q30(0.5);
  nodes[i].half_left=nodes[i].half_right=q16(300.0);
  nodes[i].shape=SH2Z_VD_SHAPE_WALLED;nodes[i].zones_by_side=1;
 }
 track.nodes=nodes;track.zones=zones;track.ramps=ramps;track.mines=mines;track.count=N;track.laps=3;
 for(int i=0;i<N;i++){
  double len=0.0,prev[3]={0,0,0},marks[3]={0,0,0};
  for(int s=0;s<=256;s++){
   int32_t pos[3];
   sh2z_vd_spline(&track,(uint32_t)i,(int32_t)((int64_t)s*(1<<30)/256),pos,NULL);
   if(s)len+=sqrt(pow(f16(pos[0])-prev[0],2)+pow(f16(pos[1])-prev[1],2)+pow(f16(pos[2])-prev[2],2));
   prev[0]=f16(pos[0]);prev[1]=f16(pos[1]);prev[2]=f16(pos[2]);
   if(s==64||s==128||s==192){marks[s/64-1]=len;for(int k=0;k<3;k++)nodes[i].mark_pos[s/64-1][k]=pos[k];}
  }
  nodes[i].length=q12(len);nodes[i].start=q12(total);
  for(int k=0;k<3;k++)nodes[i].mark_len[k]=q12(marks[k]);
  total+=len;
 }
 track.length=q12(total);
 /* A boost plate in the middle of node 2 and a recharge strip on node 9. */
 zones[0]=(sh2z_vd_zone_t){SH2Z_VD_ZONE_BOOST_PLATE,q30(0.4),q30(0.6),{q16(-100),0,0},{q16(100),0,0}};
 zones[1]=(sh2z_vd_zone_t){SH2Z_VD_ZONE_RECHARGE,q30(0.0),q30(1.0),{q16(-300),0,0},{q16(300),0,0}};
 nodes[2].zone_first=0;nodes[2].zone_count=1;
 nodes[9].zone_first=1;nodes[9].zone_count=1;
 /* A mine on the line of the first straight (span 1) and a launch ramp
  * across the back straight (span 9, travel toward -x). */
 mines[0]=(sh2z_vd_mine_t){{q16(-1500),q16(10),q16(-3000)}};
 nodes[1].mine_first=0;nodes[1].mine_count=1;
 ramps[0]=(sh2z_vd_ramp_t){{q16(1500),q16(-20),q16(2700)},
  {{q30(1.0),0,0},{0,q30(1.0),0},{0,0,q30(1.0)}},{q16(600),q16(60),q16(400)}};
 nodes[9].ramp_first=0;nodes[9].ramp_count=1;
}
static void build_vehicle(void) {
 veh=(sh2z_vd_vehicle_t){
  .force_low=q16(2.0),.force_slide=q16(1.7),.force_high=q16(0.1),.crossover=q16(33.0),
  .boost_mul=q16(2.0),.plate_mul=q16(2.0),.slide_max=q16(0.065),.slide_decay=q16(0.005),
  .slew_base=q16(1.0),.turn_free=q30(0.052),.turn_grip=q30(0.028),.top_mix=0,
  .accel_cutoff=q16(92.0),.grip_limit=q16(1.25),.slide_limit=q16(0.38),.wall_bounce=q16(1.7),
  .slide_mul=q16(1.0),.bump=q16(0.25),.slew_gain=q16(0.015),.slew_slope=0,
  .max_energy=q16(178.0),.boost_cost=q16(0.267),.recharge=q16(1.424),.hover_base=q16(5.0),
  .top_force=q16(0.198)};
 rules=(sh2z_vd_rules_t){
  .dead_zone=q30(0.1),.turn_spin=q30(0.2),.lean_turn_mix=q30(0.4),.grip_turn_mix=q30(0.125),
  .damp_side=q30(0.996),.damp_up=q30(0.94),.damp_fwd=q30(0.997),.damp_spin=q30(0.997),
  .damp_crashed=q30(0.983),.damp_rough=q30(0.987),.damp_brake=q30(0.987),
  .mine_radius=q16(30),.mine_kick=q16(15),.mine_damage=q16(12.5),.ramp_gain=q30(0.3),
  .ramp_base=q16(0.5),.ramp_decay=q30(0.8),.ramp_release=q16(0.5),.gravity=q16(1.0),
  .hover_spring=q30(0.2),.hover_top=q16(13),.hover_rise_gain=q30(0.05),.hover_crashed=q16(10),
  .double_tap=15,.side_force=q16(5),.side_fade=q16(0.8333333),.side_cut=q16(6),
  .spin_force=q16(0.5),.spin_fade=q16(0.25),.spin_cut=q16(2),.lean_force=q16(1),
  .lean_fade=q16(0.125),.lean_cut=q16(8),.lean_ice=q30(0.2),.grip_tap=30,.tap_window=27,
  .contact_grip=60,.slide_speed=q16(8),.slide_ticks=100,.turn_cost=q30(0.5),
  .lean_cost=q30(0.16),.speed_cap=q16(138.9),.boost_ticks=100,.air_road=q16(40),
  .air_walled=q16(145),.air_tunnel=q16(210),.ground_band=q16(15),.blend_band=q16(30),
  .edge_widen=q16(57.5),.edge_low=q16(-100),.edge_damage=q16(2),.tilt_follow=q30(0.19),
  .tilt_input=q30(0.15),.wall_damage=q30(0.7),.wall_min=q16(0.01),.wall_keep=q30(0.998),
  .spin_bounce=q16(2),.spin_steer=q30(0.2),.spin_ticks=150,.fall_damp=q30(0.94),
  .fall_floor=q16(-2750),.world_edge=q16(20000),.control_tick=40,.clock_tick=40,
  .hover_tick=250,.clock_num=50,.clock_den=3};
}
/* A body on the line at node 1 (a straight span), facing along the line. */
static sh2z_vd_body_t start_body(void) {
 sh2z_vd_body_t b;sh2z_basis30_t f;
 memset(&b,0,sizeof b);
 b.flags=SH2Z_VD_ACTIVE|SH2Z_VD_GRIPPING|SH2Z_VD_HOVER_SET|SH2Z_VD_CONTROLLABLE|SH2Z_VD_CAN_BOOST;
 b.node=1;b.t=q30(0.05);
 sh2z_vd_spline(&track,1,b.t,b.spos,b.fwd);
 b.fwd_mag=(int32_t)lround(65536.0*sqrt(pow(f16(b.fwd[0]),2)+pow(f16(b.fwd[1]),2)+pow(f16(b.fwd[2]),2)));
 for(int k=0;k<3;k++){b.pos[k]=b.ground[k]=b.spos[k];f.x[k]=b.fwd[k];f.y[k]=nodes[1].up[k];}
 if(sh2z_basis30_orthonormalize_forward(&f)!=0)bad("start basis",0);
 for(int k=0;k<3;k++){
  b.body[0][k]=b.road[0][k]=f.x[k];
  b.body[1][k]=b.road[1][k]=b.gravity_up[k]=b.tilt_up[k]=b.ground_up[k]=f.y[k];
  b.body[2][k]=b.road[2][k]=f.z[k];
 }
 b.lap_dist=sh2z_vd_lap_distance(&b,&track);
 b.half_left=b.half_right=q16(300);
 b.energy=veh.max_energy;b.hover=q16(8.0);b.wall_bounce=veh.wall_bounce;b.lap=1;b.laps_done=1;
 return b;
}
static sh2z_vd_input_t pad(uint16_t held,uint16_t prev,int sx) {
 sh2z_vd_input_t in={held,(uint16_t)(held&~prev),(uint16_t)(prev&~held),(int8_t)sx,0};
 return in;
}

/* ---- target memory ---- */
/* Records made only of 32-bit fields go word by word. */
static void put_words(u32 at,const void *src,size_t bytes) {
 const unsigned char *p=src;
 for(size_t i=0;i+4<=bytes;i+=4){uint32_t w;memcpy(&w,p+i,4);wl(&cpu,at+(u32)i,w);}
}
static void put_node(u32 at,const sh2z_vd_node_t *n) {
 put_words(at,n,offsetof(sh2z_vd_node_t,shape));
 wb(&cpu,at+(u32)offsetof(sh2z_vd_node_t,shape),n->shape);
 wb(&cpu,at+(u32)offsetof(sh2z_vd_node_t,zones_by_side),n->zones_by_side);
 const uint16_t h[8]={n->zone_first,n->zone_count,n->ramp_first,n->ramp_count,n->mine_first,n->mine_count,n->reserved,0};
 for(unsigned i=0;i<7;i++)ww(&cpu,at+(u32)offsetof(sh2z_vd_node_t,zone_first)+2*i,h[i]);
}
static void put_input(const sh2z_vd_input_t *in) {
 const u32 at=symc("input");
 ww(&cpu,at+(u32)offsetof(sh2z_vd_input_t,held),in->held);
 ww(&cpu,at+(u32)offsetof(sh2z_vd_input_t,pressed),in->pressed);
 ww(&cpu,at+(u32)offsetof(sh2z_vd_input_t,released),in->released);
 wb(&cpu,at+(u32)offsetof(sh2z_vd_input_t,stick_x),(u8)in->stick_x);
 wb(&cpu,at+(u32)offsetof(sh2z_vd_input_t,stick_y),(u8)in->stick_y);
}
static uint64_t words_compared;
static bool same_body(const char *name,const sh2z_vd_body_t *b) {
 const u32 at=symc(name);const unsigned char *p=(const unsigned char *)b;
 for(size_t i=0;i+4<=sizeof *b;i+=4){uint32_t w;memcpy(&w,p+i,4);if(rl(&cpu,at+(u32)i)!=w)return false;}
 words_compared+=sizeof *b/4;
 return true;
}
static void load_target(void) {
 const size_t host[8]={sizeof(sh2z_vd_node_t),sizeof(sh2z_vd_zone_t),sizeof(sh2z_vd_ramp_t),sizeof(sh2z_vd_mine_t),
  sizeof(sh2z_vd_vehicle_t),sizeof(sh2z_vd_rules_t),sizeof(sh2z_vd_body_t),sizeof(sh2z_vd_input_t)};
 for(unsigned i=0;i<8;i++)if(rl(&cpu,symc("sizes")+4*i)!=(u32)host[i])bad("record layout differs from the host",i);
 for(unsigned i=0;i<N;i++)put_node(symc("nodes")+(u32)(i*sizeof(sh2z_vd_node_t)),&nodes[i]);
 put_words(symc("zones"),zones,sizeof zones);
 put_words(symc("ramps"),ramps,sizeof ramps);
 put_words(symc("mines"),mines,sizeof mines);
 put_words(symc("vehicle"),&veh,sizeof veh);
 put_words(symc("rules"),&rules,sizeof rules);
 if(invoke("_set_track",N,(u32)track.length,(u32)track.laps,0).result!=1)bad("set_track",0);
}

/* ---- statistics ---- */
struct stats {double mean,median;unsigned min,max;double instr;};
static int cmp_u(const void*a,const void*b){unsigned x=*(const unsigned*)a,y=*(const unsigned*)b;return x<y?-1:x>y;}
static struct stats summarize(unsigned *c,size_t n,uint64_t instr) {
 qsort(c,n,sizeof *c,cmp_u);double sum=0;for(size_t i=0;i<n;i++)sum+=c[i];
 return (struct stats){sum/(double)n,n&1?c[n/2]:(c[n/2-1]+c[n/2])/2.0,c[0],c[n-1],(double)instr/(double)n};
}
static void print_stats(struct stats s){printf("{\"meanCycles\":%.2f,\"medianCycles\":%.1f,\"minCycles\":%u,\"maxCycles\":%u,\"meanInstructions\":%.2f}",s.mean,s.median,s.min,s.max,s.instr);}

/* ---- scripts ---- */
enum {MAXT=9000,MAXP=4096};
enum {S_STRAIGHT,S_STEER_BOOST,S_WALL,S_AIRBORNE,S_RACE,SCRIPTS};
static const char *const script_name[SCRIPTS]={"straight","steer_boost","wall","airborne","race"};
static const char *const script_text[SCRIPTS]={
 "ACCEL held for 600 ticks from the start line",
 "900 ticks: ACCEL, a boost every 240 ticks, LEAN_L in bursts, the stick swept -40 / 0 / +40",
 "400 ticks: ACCEL, full right stick for ticks 120-135 into the right-hand wall and on",
 "300 ticks: ACCEL from 300 units above the road, falling back onto it",
 "a line-holding driver for three laps (plate, recharge strip, mine, ramp, finish)"};
static unsigned step_cycles[SCRIPTS][MAXT],all_cycles[SCRIPTS*MAXT],loc_cycles[MAXP],off_cycles[MAXP],lap_cycles[MAXP];
static uint64_t step_instr[SCRIPTS],all_instr,loc_instr,off_instr,lap_instr;
static unsigned ticks[SCRIPTS],n_all,n_loc,n_off,n_lap,locate_every=4;
static uint64_t sr_set_ticks,cov_air,cov_recoil,cov_launch,cov_mine,cov_plate,cov_boost,cov_recharge,cov_spin,cov_falling;
static int finish_tick=-1;
static uint32_t rng=UINT32_C(0x9e3779b9);
static uint32_t rnd(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}

static void probe(const sh2z_vd_body_t *b,uint64_t it) {
 sh2z_vd_body_t h=*b;int32_t target[3];
 const bool off=n_off<MAXP&&rnd()%16u==0u;
 for(int k=0;k<3;k++)target[k]=b->pos[k];
 if(off){target[0]+=q16(9000.0);target[2]+=q16(9000.0);}
 else {target[0]+=q16((double)(rnd()%1201u)-600.0);target[1]+=q16((double)(rnd()%81u)-40.0);target[2]+=q16((double)(rnd()%1201u)-600.0);}
 const int want=sh2z_vd_locate(&h,&track,target,100,(int64_t)1<<32);
 put_words(symc("probe"),b,sizeof *b);put_words(symc("target"),target,sizeof target);
 const struct cost r=invoke("_locate",100,1,0,0);
 if((int32_t)r.result!=want)bad("locate result",it);
 if(!same_body("probe",&h))bad("locate probe body",it);
 if(want==0&&n_loc<MAXP){loc_cycles[n_loc++]=r.cycles;loc_instr+=r.instructions;}
 else if(want!=0&&n_off<MAXP){off_cycles[n_off++]=r.cycles;off_instr+=r.instructions;}
 if(want==0&&n_lap<MAXP){
  const int32_t d=sh2z_vd_lap_distance(&h,&track);
  const struct cost l=invoke("_lap_distance",0,0,0,0);
  if((int32_t)l.result!=d)bad("lap distance result",it);
  if(!same_body("probe",&h))bad("lap distance probe body",it);
  lap_cycles[n_lap++]=l.cycles;lap_instr+=l.instructions;
 }
}

static void run_script(int s) {
 sh2z_vd_body_t b=start_body();
 if(s==S_AIRBORNE)b.pos[1]+=q16(300.0);
 if(s==S_RACE)b.energy=q16(100.0);
 put_words(symc("body"),&b,sizeof b);
 uint16_t prev=0;
 const unsigned limit=s==S_STRAIGHT?600u:s==S_STEER_BOOST?900u:s==S_WALL?400u:s==S_AIRBORNE?300u:(unsigned)MAXT;
 for(unsigned i=0;i<limit;i++){
  uint16_t held=SH2Z_VD_BTN_ACCEL;int stick=0;
  switch(s){
  case S_STEER_BOOST:
   held=(uint16_t)(SH2Z_VD_BTN_ACCEL|((i%240)==200?SH2Z_VD_BTN_BOOST:0)|((i/60)%5==3?SH2Z_VD_BTN_LEAN_L:0));
   stick=((int)(i/45)%3-1)*40;break;
  case S_WALL:stick=(i>=120&&i<136)?64:0;break;
  case S_RACE:{
   double d=0.004*f16(sh2z_vd_body_lateral(&b))+
    2.5*(f30(b.body[0][0])*f30(b.road[2][0])+f30(b.body[0][1])*f30(b.road[2][1])+f30(b.body[0][2])*f30(b.road[2][2]));
   if(d>1)d=1;if(d<-1)d=-1;stick=(int)lround(d*63.0);break;}
  default:break;
  }
  const sh2z_vd_input_t in=pad(held,prev,stick);prev=held;
  sh2z_vd_step(&b,&track,&veh,&rules,&in);
  put_input(&in);
  const u32 sr=(i&1u)?2u:0u; /* every other tick with SR.S set */
  const struct cost r=call_sr("step",0,0,0,sr);
  sr_set_ticks+=sr!=0;
  if(r.result!=b.flags)bad("step flags",i);
  if(!same_body("body",&b))bad(script_name[s],i);
  step_cycles[s][ticks[s]++]=r.cycles;step_instr[s]+=r.instructions;
  all_cycles[n_all++]=r.cycles;all_instr+=r.instructions;
  cov_air+=(b.flags&SH2Z_VD_AIRBORNE)!=0;cov_recoil+=(b.flags&SH2Z_VD_RECOIL)!=0;
  cov_launch+=(b.flags&SH2Z_VD_LAUNCHED)!=0;cov_mine+=(b.flags&SH2Z_VD_MINE_HIT)!=0;
  cov_plate+=(b.flags&SH2Z_VD_PLATE_BOOST)!=0;cov_boost+=b.boost_timer>0;
  cov_recharge+=(b.flags&SH2Z_VD_ZONE_MASK)==SH2Z_VD_ZONE_RECHARGE;
  cov_spin+=(b.flags&SH2Z_VD_SPINNING)!=0;cov_falling+=(b.flags&SH2Z_VD_FALLING)!=0;
  if(i%locate_every==0u)probe(&b,i);
  if(b.flags&SH2Z_VD_FINISHED){if(s==S_RACE&&finish_tick<0)finish_tick=(int)i;if(s==S_RACE)break;}
 }
}

int main(int argc,char**argv) {
 if(argc<2){fprintf(stderr,"usage: vehicle_dynamics_harness model.elf [locate-every]\n");return 2;}
 if(argc>2){locate_every=(unsigned)strtoul(argv[2],0,10);if(!locate_every)return 2;}
 load_elf_symbols(argv[1]);load_elf(argv[1]);
 cpu.model=SHMT_SH2;cpu.MappedMemoryReadByte=rb;cpu.MappedMemoryReadWord=rw;cpu.MappedMemoryReadLong=rl;cpu.MappedMemoryWriteByte=wb;cpu.MappedMemoryWriteWord=ww;cpu.MappedMemoryWriteLong=wl;
 for(unsigned i=0;i<65536;i++)cpu.opcodes[i]=decode(SHMT_SH2,(u16)i);for(unsigned i=0;i<256;i++)cpu.fetchlist[i]=fetch;
 build_track();build_vehicle();load_target();
 for(int s=0;s<SCRIPTS;s++)run_script(s);
 if(finish_tick<0)bad("the race script did not finish",0);
 uint64_t total=0;for(int s=0;s<SCRIPTS;s++)total+=ticks[s];
 printf("{\"schema\":1,\"kind\":\"Yabause isolated SH2 interpreter flat-RAM model, not Saturn timing\",");
 printf("\"verified\":{\"ticks\":%"PRIu64",\"ticksSrSet\":%"PRIu64",\"bodyWordsCompared\":%"PRIu64",\"locateProbes\":%u,\"locateOffLine\":%u,\"lapDistanceProbes\":%u,"
  "\"entries\":\"every tick of every script: the step's return and all %u words of the SH-2 body against the host build of the same source; every probe: sh2z_vd_locate's result and all probe words, then sh2z_vd_lap_distance's result and all probe words, against the host\"},",
  total,sr_set_ticks,words_compared,n_loc,n_off,n_lap,(unsigned)(sizeof(sh2z_vd_body_t)/4));
 printf("\"coverage\":{\"airborneTicks\":%"PRIu64",\"wallTicks\":%"PRIu64",\"launchedTicks\":%"PRIu64",\"mineHitTicks\":%"PRIu64",\"plateBoostTicks\":%"PRIu64",\"boostTicks\":%"PRIu64",\"rechargeTicks\":%"PRIu64",\"spinningTicks\":%"PRIu64",\"fallingTicks\":%"PRIu64",\"raceFinishTick\":%d},",
  cov_air,cov_recoil,cov_launch,cov_mine,cov_plate,cov_boost,cov_recharge,cov_spin,cov_falling,finish_tick);
 printf("\"rows\":{");
 for(int s=0;s<SCRIPTS;s++){
  if(s)putchar(',');
  printf("\"%s\":{\"description\":\"%s\",\"ticks\":%u,\"step\":",script_name[s],script_text[s],ticks[s]);
  print_stats(summarize(step_cycles[s],ticks[s],step_instr[s]));printf("}");
 }
 printf("},\"step\":");print_stats(summarize(all_cycles,n_all,all_instr));
 printf(",\"locate\":");print_stats(summarize(loc_cycles,n_loc,loc_instr));
 printf(",\"locateOffLine\":");print_stats(summarize(off_cycles,n_off,off_instr));
 printf(",\"lapDistance\":");print_stats(summarize(lap_cycles,n_lap,lap_instr));
 printf("}\n");
 return 0;
}
