/* Independent oracle, edge cases, error study and negative controls for the
 * view-contract-1 batch. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;

#define POSES 512u
#define PER_POSE 2048u

static uint64_t checks;
static uint32_t state=UINT32_C(0x5eed1234);

static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"view check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {
    state^=state<<13;state^=state>>17;state^=state<<5;return state;
}
static int32_t random_range(int32_t lo,int32_t hi) {
    return lo+(int32_t)(random_word()%(uint32_t)(hi-lo+1));
}

/* Oracle mutations, used only by the negative controls. */
enum { MUT_NONE=0, MUT_FLOOR_DIVIDE=1, MUT_SWAP_LEFT_RIGHT=2, MUT_NEAR_INCLUSIVE=4 };

static wide_t floor_div_wide(wide_t n,wide_t d) {
    wide_t q=n/d;
    if((n%d)!=0 && ((n<0)!=(d<0)))--q;
    return q;
}
static wide_t clamp_wide(wide_t a,wide_t lo,wide_t hi) {return a<lo?lo:a>hi?hi:a;}

static sh2z_view_vertex_t oracle_vertex(const sh2z_view_t *v,const int16_t p[3],unsigned mutation) {
    sh2z_view_vertex_t o;
    memset(&o,0,sizeof(o));
    wide_t view[3];
    for(unsigned i=0;i<3;++i) {
        view[i]=v->t[i];
        for(unsigned j=0;j<3;++j)view[i]+=(wide_t)v->r[i][j]*p[j];
        check(view[i]>=INT32_MIN && view[i]<=INT32_MAX,"bound proof keeps view in int32");
    }
    o.x=(int32_t)view[0];o.y=(int32_t)view[1];o.z=(int32_t)view[2];
    const bool near=(mutation&MUT_NEAR_INCLUSIVE)?view[2]<=v->near_z:view[2]<v->near_z;
    if(near) {o.outcode=SH2Z_OUT_NEAR;return o;}
    const wide_t nx=(wide_t)v->focal_x*view[0],ny=(wide_t)v->focal_y*view[1];
    wide_t qx,qy;
    if(mutation&MUT_FLOOR_DIVIDE) {qx=floor_div_wide(nx,view[2]);qy=floor_div_wide(ny,view[2]);}
    else {
        /* Truncation from magnitudes, independent of the C '/' rule. */
        const wide_t ax=nx<0?-nx:nx,ay=ny<0?-ny:ny;
        qx=(nx<0?-1:1)*(ax/view[2]);qy=(ny<0?-1:1)*(ay/view[2]);
    }
    qx=clamp_wide(qx,INT32_MIN,INT32_MAX);qy=clamp_wide(qy,INT32_MIN,INT32_MAX);
    o.sx=(int16_t)clamp_wide(v->center_x+floor_div_wide(qx,65536),INT16_MIN,INT16_MAX);
    o.sy=(int16_t)clamp_wide(v->center_y-floor_div_wide(qy,65536),INT16_MIN,INT16_MAX);
    unsigned code=0;
    if(view[2]>v->far_z)code|=SH2Z_OUT_FAR;
    unsigned left=SH2Z_OUT_LEFT,right=SH2Z_OUT_RIGHT;
    if(mutation&MUT_SWAP_LEFT_RIGHT){left=SH2Z_OUT_RIGHT;right=SH2Z_OUT_LEFT;}
    if(o.sx<v->clip[0])code|=left;
    if(o.sx>v->clip[2])code|=right;
    if(o.sy<v->clip[1])code|=SH2Z_OUT_TOP;
    if(o.sy>v->clip[3])code|=SH2Z_OUT_BOTTOM;
    o.outcode=(uint8_t)code;
    return o;
}

static int16_t q14(double x) {
    const double s=floor(x*16384.0+0.5);
    return (int16_t)(s>32767?32767:s<-32768?-32768:s);
}
/* Rotation rows Rz(roll) * Rx(pitch) * Ry(yaw), composed numerically. */
static void multiply(double out[3][3],const double a[3][3],const double b[3][3]) {
    double m[3][3];
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)
        m[i][j]=a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j];
    memcpy(out,m,sizeof(m));
}
static void rotation(double out[3][3],double yaw,double pitch,double roll) {
    const double y[3][3]={{cos(yaw),0,-sin(yaw)},{0,1,0},{sin(yaw),0,cos(yaw)}};
    const double x[3][3]={{1,0,0},{0,cos(pitch),sin(pitch)},{0,-sin(pitch),cos(pitch)}};
    const double z[3][3]={{cos(roll),sin(roll),0},{-sin(roll),cos(roll),0},{0,0,1}};
    double xy[3][3];
    multiply(xy,x,y);multiply(out,z,(const double (*)[3])xy);
}
static sh2z_view_t random_view(double r[3][3],double eye[3]) {
    rotation(r,(random_word()%65536)*6.283185307179586/65536.0,
        ((double)(random_word()%65536)/65536.0-0.5)*3.0,
        ((double)(random_word()%65536)/65536.0-0.5)*1.2);
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)v.r[i][j]=q14(r[i][j]);
    v.focal_x=random_range(64,1024)*65536+random_range(0,65535);
    v.focal_y=random_range(64,1024)*65536+random_range(0,65535);
    v.center_x=(int16_t)random_range(0,352);v.center_y=(int16_t)random_range(0,256);
    v.near_z=random_range(1,16*16384);v.far_z=v.near_z+random_range(1,INT32_MAX-v.near_z);
    v.clip[0]=(int16_t)random_range(-64,160);v.clip[2]=(int16_t)random_range(v.clip[0],400);
    v.clip[1]=(int16_t)random_range(-64,120);v.clip[3]=(int16_t)random_range(v.clip[1],300);
    sh2z_view_t valid;
    check(sh2z_view_init(&valid,&v),"random rotation view is valid");
    sh2z_vec3_t e={random_range(-30000,30000)*65536+random_range(0,65535),
        random_range(-30000,30000)*65536+random_range(0,65535),
        random_range(-30000,30000)*65536+random_range(0,65535)};
    check(sh2z_view_place(&valid,e),"eye inside int16 world is placeable");
    eye[0]=e.x/65536.0;eye[1]=e.y/65536.0;eye[2]=e.z/65536.0;
    return valid;
}
static void random_positions(sh2z_pos16_t *p,size_t n,const double eye[3]) {
    for(size_t k=0;k<n;++k) {
        const unsigned mode=random_word()%4u;
        int32_t c[3];
        for(unsigned j=0;j<3;++j) {
            if(mode==0)c[j]=random_range(INT16_MIN,INT16_MAX);
            else {
                const int32_t span=mode==1?16:mode==2?2048:32768;
                c[j]=(int32_t)floor(eye[j])+random_range(-span,span);
            }
            c[j]=c[j]<INT16_MIN?INT16_MIN:c[j]>INT16_MAX?INT16_MAX:c[j];
        }
        p[k].x=(int16_t)c[0];p[k].y=(int16_t)c[1];p[k].z=(int16_t)c[2];
    }
}

static sh2z_pos16_t positions[PER_POSE];
static sh2z_view_vertex_t got[PER_POSE],want[PER_POSE];

/* Returns the number of mismatching vertices across the sweep. */
static uint64_t sweep(unsigned mutation,uint64_t *vertices,uint64_t *near,uint64_t *saturated) {
    state=UINT32_C(0x5eed1234);
    uint64_t mismatches=0;
    for(unsigned pose=0;pose<POSES;++pose) {
        double r[3][3],eye[3];
        const sh2z_view_t v=random_view(r,eye);
        random_positions(positions,PER_POSE,eye);
        check(sh2z_view_batch(got,&v,positions,sizeof(positions[0]),PER_POSE),"batch succeeds");
        for(unsigned k=0;k<PER_POSE;++k) {
            const int16_t p[3]={positions[k].x,positions[k].y,positions[k].z};
            want[k]=oracle_vertex(&v,p,mutation);
            if(memcmp(&got[k],&want[k],sizeof(got[k])))++mismatches;
            if(vertices)++*vertices;
            if(near && (got[k].outcode&SH2Z_OUT_NEAR))++*near;
            if(saturated && (got[k].sx==INT16_MIN || got[k].sx==INT16_MAX))++*saturated;
        }
    }
    return mismatches;
}

static sh2z_view_t identity_view(void) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=v.focal_y=65536;
    v.near_z=5*16384;v.far_z=1000*16384;
    v.clip[0]=-10;v.clip[1]=-20;v.clip[2]=30;v.clip[3]=40;
    return v;
}
static sh2z_view_vertex_t one(const sh2z_view_t *v,int16_t x,int16_t y,int16_t z) {
    const sh2z_pos16_t p={x,y,z};
    sh2z_view_vertex_t o;
    check(sh2z_view_batch(&o,v,&p,sizeof(p),1),"single vertex succeeds");
    const int16_t q[3]={x,y,z};
    const sh2z_view_vertex_t w=oracle_vertex(v,q,MUT_NONE);
    check(!memcmp(&o,&w,sizeof(o)),"single vertex matches oracle");
    return o;
}

static void edge_cases(void) {
    const sh2z_view_t v=identity_view();
    /* Exactly on the near plane is in front; one unit closer is NEAR. */
    check(one(&v,0,0,5).outcode==0,"z == near is not NEAR");
    sh2z_view_vertex_t o=one(&v,3,4,4);
    check(o.outcode==SH2Z_OUT_NEAR && o.sx==0 && o.sy==0 && o.x==3*16384,"z < near is NEAR, screen zero");
    check(one(&v,0,0,-7).outcode==SH2Z_OUT_NEAR,"behind camera is NEAR");
    /* Each clip edge, one pixel either side. focal 1, z 1 gives sx == x. */
    sh2z_view_t unit=v;unit.near_z=16384;
    check(one(&unit,-11,0,1).outcode==SH2Z_OUT_LEFT,"left - 1");
    check(one(&unit,-10,0,1).outcode==0,"left");
    check(one(&unit,30,0,1).outcode==0,"right");
    check(one(&unit,31,0,1).outcode==SH2Z_OUT_RIGHT,"right + 1");
    check(one(&unit,0,21,1).outcode==SH2Z_OUT_TOP,"top - 1 (y up)");
    check(one(&unit,0,20,1).outcode==0,"top");
    check(one(&unit,0,-40,1).outcode==0,"bottom");
    check(one(&unit,0,-41,1).outcode==SH2Z_OUT_BOTTOM,"bottom + 1");
    check(one(&unit,0,0,1000).outcode==0 && one(&unit,0,0,1001).outcode==SH2Z_OUT_FAR,"far boundary");
    /* Quotient and screen saturation on both signs, z of one unit. */
    sh2z_view_t big=unit;big.focal_x=big.focal_y=INT32_MAX;
    o=one(&big,INT16_MAX,INT16_MIN,1);
    check(o.sx==INT16_MAX && o.sy==INT16_MAX,"positive x and negative y saturate");
    o=one(&big,INT16_MIN,INT16_MAX,1);
    check(o.sx==INT16_MIN && o.sy==-INT16_MAX,"negative x saturates; positive y gives center - 32767");
    check((o.outcode&(SH2Z_OUT_LEFT|SH2Z_OUT_TOP))==(SH2Z_OUT_LEFT|SH2Z_OUT_TOP),"saturated vertex keeps side bits");
    /* Truncation toward zero for negative non-exact quotients. */
    sh2z_view_t third=unit;third.focal_x=65536;
    o=one(&third,-1,0,3);
    check(o.sx==-1,"trunc(-65536/3) = -21845 then floor to -1");

    /* Bound proof: exactly at the limit is valid, one more is not. */
    sh2z_view_t edge=identity_view();
    edge.r[0][0]=32767;edge.r[0][1]=32767;edge.r[0][2]=0;
    edge.t[0]=INT32_MAX-(int32_t)(UINT32_C(32768)*65534u);
    sh2z_view_t checked;
    check(sh2z_view_init(&checked,&edge),"bound exactly met");
    o=one(&checked,INT16_MIN,INT16_MIN,10);
    edge.t[0]+=1;
    memset(&checked,0x5a,sizeof(checked));
    sh2z_view_t before=checked;
    check(!sh2z_view_init(&checked,&edge),"bound exceeded by one");
    check(!memcmp(&checked,&before,sizeof(checked)),"failed init leaves output unchanged");
    edge.t[0]-=1;edge.t[0]=-edge.t[0];
    check(sh2z_view_init(&checked,&edge),"negative translation at the bound");
    check(sh2z_view_init(&edge,&edge),"init may alias");

    /* Invalid views. */
    sh2z_view_t bad=identity_view();bad.focal_x=0;check(!sh2z_view_init(&checked,&bad),"focal_x > 0");
    bad=identity_view();bad.focal_y=-1;check(!sh2z_view_init(&checked,&bad),"focal_y > 0");
    bad=identity_view();bad.near_z=0;check(!sh2z_view_init(&checked,&bad),"near > 0");
    bad=identity_view();bad.far_z=bad.near_z;check(!sh2z_view_init(&checked,&bad),"far > near");
    bad=identity_view();bad.clip[0]=31;check(!sh2z_view_init(&checked,&bad),"left <= right");
    bad=identity_view();bad.clip[1]=41;check(!sh2z_view_init(&checked,&bad),"top <= bottom");
    check(!sh2z_view_init(NULL,&v) && !sh2z_view_init(&checked,NULL),"null pointers");

    /* place: exact floor, failure leaves the view unchanged. */
    sh2z_view_t placed=identity_view();
    check(sh2z_view_place(&placed,(sh2z_vec3_t){3*65536+1,-2*65536-1,0}),"place identity");
    check(placed.t[0]==-(3*16384) && placed.t[1]==2*16384+1 && placed.t[2]==0,"t = -floor(r*eye/65536)");
    placed=identity_view();placed.r[0][0]=32767;placed.r[0][1]=32767;
    before=placed;
    check(!sh2z_view_place(&placed,(sh2z_vec3_t){INT32_MAX,INT32_MAX,0}),"place rejects out-of-bound t");
    check(!memcmp(&placed,&before,sizeof(placed)),"failed place leaves view unchanged");

    /* Argument contract: nothing written on failure. */
    sh2z_pos16_t p[4]={{1,2,10},{3,4,10},{5,6,10},{7,8,10}};
    sh2z_view_vertex_t out[4],sentinel[4];
    memset(sentinel,0xa5,sizeof(sentinel));memcpy(out,sentinel,sizeof(out));
    check(sh2z_view_batch(NULL,&v,NULL,0,0) && sh2z_view_batch(out,&v,p,7,0),"count 0 succeeds");
    check(!sh2z_view_batch(out,&v,p,5,2),"stride below 6");
    check(!sh2z_view_batch(out,&v,p,7,2),"odd stride");
    check(!sh2z_view_batch(out,&v,(const char *)p+1,6,2),"misaligned positions");
    check(!sh2z_view_batch(NULL,&v,p,6,2) && !sh2z_view_batch(out,&v,NULL,6,2),"null with count");
    check(!sh2z_view_batch(out,NULL,p,6,2) && !sh2z_view_batch(out,&bad,p,6,2),"invalid view");
    check(!memcmp(out,sentinel,sizeof(out)),"failed calls write nothing");
    union {sh2z_view_vertex_t v[4];sh2z_pos16_t p[13];} shared;
    memset(&shared,0,sizeof(shared));
    check(!sh2z_view_batch(shared.v,&v,shared.p+4,6,2),"overlapping output rejected");
    /* Stride larger than a position: records with trailing data. */
    struct {int16_t x,y,z,u,w;} records[3]={{1,2,10,99,99},{-4,5,20,99,99},{6,-7,6,99,99}};
    check(sh2z_view_batch(out,&v,records,sizeof(records[0]),3),"stride 10");
    for(unsigned k=0;k<3;++k) {
        const int16_t q[3]={records[k].x,records[k].y,records[k].z};
        const sh2z_view_vertex_t w=oracle_vertex(&v,q,MUT_NONE);
        check(!memcmp(&out[k],&w,sizeof(w)),"stride 10 matches oracle");
        check(!out[k].reserved[0] && !out[k].reserved[1] && !out[k].reserved[2],"reserved bytes zero");
    }
}

/* Screen error against double precision, using the unquantized rotation and
 * eye. Only vertices inside the clip rectangle and at least one unit deep. */
static void error_study(void) {
    state=UINT32_C(0x0e4404);
    static const double bands[5]={1,16,256,4096,65536};
    double worst[4]={0,0,0,0},edge[4]={0,0,0,0};
    uint64_t counted[4]={0,0,0,0},off[4]={0,0,0,0};
    for(unsigned pose=0;pose<POSES;++pose) {
        double r[3][3],eye[3];
        sh2z_view_t v=random_view(r,eye);
        v.near_z=16384;
        check(sh2z_view_init(&v,&v),"study view");
        /* Aim each position at a random pixel inside the clip rectangle at a
         * random depth, then map it back to the int16 world grid. */
        unsigned n=0;
        for(unsigned k=0;k<PER_POSE;++k) {
            const double z=pow(2.0,(double)(random_word()%65536)/65536.0*16.0);
            const double px=v.clip[0]+(double)(random_word()%65536)/65536.0*(v.clip[2]-v.clip[0]);
            const double py=v.clip[1]+(double)(random_word()%65536)/65536.0*(v.clip[3]-v.clip[1]);
            const double view[3]={(px-v.center_x)*z/(v.focal_x/65536.0),(v.center_y-py)*z/(v.focal_y/65536.0),z};
            double world[3];
            bool ok=true;
            for(unsigned j=0;j<3;++j) {
                world[j]=floor(eye[j]+r[0][j]*view[0]+r[1][j]*view[1]+r[2][j]*view[2]+0.5);
                if(world[j]<INT16_MIN || world[j]>INT16_MAX)ok=false;
            }
            if(!ok)continue;
            positions[n].x=(int16_t)world[0];positions[n].y=(int16_t)world[1];positions[n].z=(int16_t)world[2];++n;
        }
        check(sh2z_view_batch(got,&v,positions,sizeof(positions[0]),n),"study batch");
        for(unsigned k=0;k<n;++k) {
            if(got[k].outcode)continue;
            const double p[3]={positions[k].x-eye[0],positions[k].y-eye[1],positions[k].z-eye[2]};
            double view[3];
            for(unsigned i=0;i<3;++i)view[i]=r[i][0]*p[0]+r[i][1]*p[1]+r[i][2]*p[2];
            if(view[2]<1.0)continue;
            const double sx=v.center_x+v.focal_x/65536.0*view[0]/view[2];
            const double sy=v.center_y-v.focal_y/65536.0*view[1]/view[2];
            const double e=fmax(fabs(got[k].sx-floor(sx)),fabs(got[k].sy-ceil(sy)));
            /* Distance of the exact value from the nearest pixel boundary. */
            const double d=fmin(fabs(sx-floor(sx+0.5)),fabs(sy-floor(sy+0.5)));
            for(unsigned b=0;b<4;++b)if(view[2]>=bands[b] && view[2]<bands[b+1]) {
                if(e>worst[b])worst[b]=e;
                if(e>0) {++off[b];if(d>edge[b])edge[b]=d;}
                ++counted[b];
            }
        }
    }
    printf("error study (pixels vs double, inside clip):\n");
    for(unsigned b=0;b<4;++b) {
        printf("  depth %6.0f..%6.0f units: %8llu vertices, %6llu off by one, max error %.0f px,"
            " off-by-one at most %.4f px from a pixel edge\n",bands[b],bands[b+1],
            (unsigned long long)counted[b],(unsigned long long)off[b],worst[b],edge[b]);
        check(counted[b]>10000,"study samples every depth band");
        check(worst[b]<=1.0,"screen error at most one pixel");
        /* Q1.14 rotation and Q.14 eye rounding move a point by well under a
         * quarter pixel at focal lengths up to 1024 px. */
        check(edge[b]<0.25,"off-by-one only within a quarter pixel of an edge");
    }
}

int main(void) {
    uint64_t vertices=0,near=0,saturated=0;
    check(sweep(MUT_NONE,&vertices,&near,&saturated)==0,"batch equals oracle on the sweep");
    printf("sweep: %llu vertices exact (%llu NEAR, %llu saturated sx)\n",
        (unsigned long long)vertices,(unsigned long long)near,(unsigned long long)saturated);
    check(near>0 && saturated>0,"sweep reaches NEAR and saturation");
    edge_cases();
    error_study();
    /* Negative controls: each mutated oracle must disagree somewhere. */
    const unsigned mutations[3]={MUT_FLOOR_DIVIDE,MUT_SWAP_LEFT_RIGHT,MUT_NEAR_INCLUSIVE};
    const char *names[3]={"floor instead of trunc","LEFT/RIGHT swapped","NEAR inclusive"};
    for(unsigned m=0;m<3;++m) {
        uint64_t bad=sweep(mutations[m],NULL,NULL,NULL);
        const sh2z_view_t v=identity_view();
        const int16_t on_near[3]={0,0,5},negative[3]={-1,0,3};
        sh2z_view_t unit=v;unit.near_z=16384;
        sh2z_view_vertex_t a=one(&v,0,0,5),b=one(&unit,-1,0,3);
        const sh2z_view_vertex_t ma=oracle_vertex(&v,on_near,mutations[m]);
        const sh2z_view_vertex_t mb=oracle_vertex(&unit,negative,mutations[m]);
        bad+=memcmp(&a,&ma,sizeof(a))!=0;
        bad+=memcmp(&b,&mb,sizeof(b))!=0;
        printf("control '%s': %llu mismatches\n",names[m],(unsigned long long)bad);
        check(bad>0,"mutated oracle is detected");
    }
    printf("view checks: %llu passed\n",(unsigned long long)checks);
    return 0;
}
