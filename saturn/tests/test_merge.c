/* Independent oracle, properties, edge cases and negative controls for the
 * view-merge-1 bounded texture-merge planner. MIT; SH2ZAM Contributors. */
#include "sh2zam/view_experimental.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__extension__ typedef __int128 wide_t;
enum { MAXB=160 };

static uint64_t checks;
static uint32_t state=UINT32_C(0x3e7a0001);
static void check(bool value,const char *what) {
    ++checks;
    if(!value) {
        fprintf(stderr,"merge check %llu failed: %s\n",(unsigned long long)checks,what);
        exit(EXIT_FAILURE);
    }
}
static uint32_t random_word(void) {state^=state<<13;state^=state>>17;state^=state<<5;return state;}
static int32_t random_range(int32_t lo,int32_t hi) {
    return (int32_t)((int64_t)lo+(int64_t)(random_word()%(uint64_t)((int64_t)hi-lo+1)));
}
static double frand(void) {return (double)random_word()/4294967296.0;}
static double frange(double lo,double hi) {return lo+(hi-lo)*frand();}
static wide_t absw(wide_t a) {return a<0?-a:a;}

typedef struct {
    unsigned count;
    sh2z_view_vertex_t left[MAXB],right[MAXB];
    uint32_t rows[MAXB],ends[MAXB];
    bool straight;   /* rails are straight lines in 3D */
} strip_t;

/* ---- oracle, written from the header text in 128-bit arithmetic ---------- */
enum { MUT_NONE=0, MUT_INTERIOR=1, MUT_NO_RIGHT=2, MUT_NO_GUARD=4, MUT_SMALLEST=8, MUT_ENDS=16 };
#define BLOCKING (~(unsigned)(SH2Z_OUT_LEFT|SH2Z_OUT_RIGHT|SH2Z_OUT_TOP|SH2Z_OUT_BOTTOM)&0xffu)

/* Row t of the run p0 .. pn (S rows) at the boundary pk. Rows mode uses the
 * expanded E . D = S * ((Pk - P0) . D) - t * |D|^2. */
static bool oracle_point(const sh2z_view_vertex_t *p0,const sh2z_view_vertex_t *pk,const sh2z_view_vertex_t *pn,
    wide_t s,wide_t t,uint32_t tol,unsigned mode,unsigned mutation) {
    const wide_t ax=pk->sx-p0->sx,ay=pk->sy-p0->sy,dx=pn->sx-p0->sx,dy=pn->sy-p0->sy;
    const wide_t ex=absw(s*ax-t*dx),ey=absw(s*ay-t*dy),m=ex>ey?ex:ey;
    if(mode==SH2Z_VIEW_MERGE_PIXELS)return m*256<=(wide_t)tol*s;
    if(!(mutation&MUT_NO_GUARD) && m>s)return false;
    const wide_t d2=dx*dx+dy*dy;
    return absw(s*(ax*dx+ay*dy)-t*d2)*256<=(wide_t)tol*d2;
}
static bool oracle_run(const strip_t *st,unsigned i,unsigned r,uint32_t tol,unsigned mode,unsigned mutation) {
    const wide_t s=(wide_t)st->rows[i+r]-st->rows[i];
    const unsigned end=(mutation&MUT_INTERIOR)?r-1u:r;
    for(unsigned k=1;k<end;++k) {
        const wide_t t=(wide_t)st->rows[i+k]-st->rows[i];
        if(!oracle_point(&st->left[i],&st->left[i+k],&st->left[i+r],s,t,tol,mode,mutation))return false;
        if(!(mutation&MUT_NO_RIGHT) && !oracle_point(&st->right[i],&st->right[i+k],&st->right[i+r],s,t,tol,mode,mutation))return false;
    }
    return true;
}
static bool oracle_allowed(const strip_t *st,unsigned i,unsigned r,uint32_t mask,const uint32_t *ends,unsigned mutation) {
    if(r<2u || r>31u || i+r>st->count-1u || !((mask>>r)&1u))return false;
    if(ends && !((ends[i+r]>>r)&1u))return false;
    const unsigned from=(mutation&MUT_ENDS)?i+1u:i,to=(mutation&MUT_ENDS)?i+r-1u:i+r;
    for(unsigned j=from;j<=to;++j)if((st->left[j].outcode|st->right[j].outcode)&BLOCKING)return false;
    return true;
}
static int oracle_plan(uint8_t *out,const strip_t *st,uint32_t mask,const uint32_t *ends,uint32_t tol,unsigned mode,unsigned mutation) {
    unsigned i=0;
    int n=0;
    while(i<st->count-1u) {
        unsigned best=1;
        for(unsigned step=0;step<30u;++step) {
            const unsigned r=(mutation&MUT_SMALLEST)?2u+step:31u-step;
            if(oracle_allowed(st,i,r,mask,ends,mutation) && oracle_run(st,i,r,tol,mode,mutation)){best=r;break;}
        }
        out[n++]=(uint8_t)best;
        i+=best;
    }
    return n;
}

/* ---- strip generators ----------------------------------------------------- */
static sh2z_view_t screen_view(double focal) {
    sh2z_view_t v;
    memset(&v,0,sizeof(v));
    v.r[0][0]=v.r[1][1]=v.r[2][2]=16384;
    v.focal_x=v.focal_y=(int32_t)(focal*65536.0);
    v.center_x=160;v.center_y=112;
    v.near_z=4096;v.far_z=4000*16384;
    v.clip[0]=0;v.clip[1]=0;v.clip[2]=319;v.clip[3]=223;
    check(sh2z_view_init(&v,&v),"screen view valid");
    return v;
}
static int32_t q14(double a) {
    const double s=floor(a*16384.0+0.5);
    return (int32_t)(s>2147483647.0?2147483647.0:s<-2147483648.0?-2147483648.0:s);
}
static void fill_rows(strip_t *st,bool huge) {
    const unsigned n=st->count;
    const uint32_t base=1u<<(3u+random_word()%5u);
    uint64_t r=random_word()%4u==0?(random_word()&0x7fffffffu):0u;
    if(huge) {   /* S close to 2^32 - 1 overall */
        r=random_word()%64u;
        const uint64_t each=(UINT64_C(0xffffffff)-r)/(n-1u);
        st->rows[0]=(uint32_t)r;
        for(unsigned k=1;k<n;++k){r+=each-(random_word()%2u?random_word()%(uint32_t)(each/8u+1u):0u);st->rows[k]=(uint32_t)r;}
        return;
    }
    st->rows[0]=(uint32_t)r;
    for(unsigned k=1;k<n;++k) {
        uint32_t span=base;
        if(k==1u && random_word()%10u<3u)span=1u+random_word()%base;            /* partial first repetition */
        else if(k==n-1u && random_word()%10u<3u)span=1u+random_word()%base;     /* partial last repetition */
        else if(random_word()%10u==0u)span=1u+random_word()%(2u*base);           /* odd span */
        r+=span;
        st->rows[k]=(uint32_t)r;
    }
}
/* A road seen from a synthetic camera: a centre line with curvature, hills
 * and banking, two rails, projected by the library. */
static void road_strip(strip_t *st) {
    const double focal=frange(120.0,320.0);
    const sh2z_view_t v=screen_view(focal);
    st->count=random_word()%2u?64u:2u+random_word()%95u;
    const double yaw=frange(-0.5,0.5),pitch=frange(-0.35,0.05),roll=random_word()%5u<2u?frange(-0.3,0.3):0.0;
    const double h=frange(0.3,8.0),s0=random_word()%7u==0u?frange(-20.0,0.0):exp(frange(log(0.3),log(300.0)));
    const double rep=exp(frange(log(0.2),log(12.0))),w=frange(0.5,12.0),off=frange(-10.0,10.0);
    st->straight=random_word()%3u==0u;
    const double kappa=st->straight?0.0:frange(-0.06,0.06),dk=st->straight?0.0:frange(-0.002,0.002);
    const double slope=frange(-0.15,0.15),hill_a=st->straight?0.0:frange(0.0,3.0),hill_f=frange(0.01,0.2);
    const double bank=random_word()%2u?frange(-0.4,0.4):0.0;
    const double cy=cos(yaw),sy=sin(yaw),cp=cos(pitch),sp=sin(pitch),cr=cos(roll),sr=sin(roll);
    sh2z_view_vertex_t lv[MAXB],rv[MAXB];
    memset(lv,0,sizeof(lv));memset(rv,0,sizeof(rv));
    /* Centre line by midpoint integration of the heading from s = 0. */
    double cx=off,cz=0.0,s=0.0;
    for(unsigned k=0;k<st->count;++k) {
        const double target=s0+rep*k;
        const unsigned steps=k?16u:64u;
        const double ds=(target-s)/steps;
        for(unsigned q=0;q<steps;++q) {
            const double m=s+ds*(q+0.5),th=kappa*m+0.5*dk*m*m;
            cx+=ds*sin(th);cz+=ds*cos(th);
        }
        s=target;
        const double th=kappa*s+0.5*dk*s*s,ccy=-h+slope*s+hill_a*sin(hill_f*s);
        for(unsigned side=0;side<2u;++side) {
            const double sign=side?-1.0:1.0;   /* left rail at +w along (-cos, 0, sin) */
            const double p[3]={cx-sign*w*cos(bank)*cos(th),ccy+sign*w*sin(bank),cz+sign*w*cos(bank)*sin(th)};
            /* view = Rz(roll) Rx(pitch) Ry(yaw) p */
            const double a0=cy*p[0]-sy*p[2],a2=sy*p[0]+cy*p[2];
            const double b1=cp*p[1]+sp*a2,b2=-sp*p[1]+cp*a2;
            const double vx=cr*a0+sr*b1,vy=-sr*a0+cr*b1;
            sh2z_view_vertex_t *o=side?&rv[k]:&lv[k];
            o->x=q14(vx);o->y=q14(vy);o->z=q14(b2);
        }
    }
    check(sh2z_view_project(st->left,&v,lv,st->count) && sh2z_view_project(st->right,&v,rv,st->count),"road projection");
    fill_rows(st,random_word()%16u==0u);
}
static int16_t extreme16(void) {
    switch(random_word()%4u) {
    case 0: return (int16_t)random_word();
    case 1: return random_word()&1u?INT16_MAX:INT16_MIN;
    case 2: return (int16_t)(random_word()&1u?INT16_MAX-random_range(0,2):INT16_MIN+random_range(0,2));
    default: return (int16_t)random_range(-2000,2000);
    }
}
/* Adversarial strips: int16 extremes, nearly linear rails with huge row
 * counts (the test sits right at its threshold), random clip flags. */
static void adversarial_strip(strip_t *st) {
    st->count=2u+random_word()%(MAXB-1u);
    st->straight=false;
    memset(st->left,0,sizeof(st->left));memset(st->right,0,sizeof(st->right));
    const unsigned kind=random_word()%3u;
    fill_rows(st,random_word()%2u==0u);
    if(kind==2u){const uint32_t r0=st->rows[0]>>1;for(unsigned k=0;k<st->count;++k)st->rows[k]=k+r0;}   /* one row per repetition */
    for(unsigned side=0;side<2u;++side) {
        sh2z_view_vertex_t *p=side?st->right:st->left;
        if(kind==0u) {
            for(unsigned k=0;k<st->count;++k){p[k].sx=extreme16();p[k].sy=extreme16();}
        } else {
            /* Linear in the row from one int16 corner to another, plus 0 or +-1. */
            const double ax=extreme16(),ay=extreme16(),bx=extreme16(),by=extreme16();
            const double total=(double)st->rows[st->count-1]-st->rows[0];
            for(unsigned k=0;k<st->count;++k) {
                const double f=((double)st->rows[k]-st->rows[0])/total;
                double x=floor(ax+(bx-ax)*f+0.5),y=floor(ay+(by-ay)*f+0.5);
                if(random_word()%3u==0u)x+=(double)random_range(-1,1);
                if(random_word()%3u==0u)y+=(double)random_range(-1,1);
                p[k].sx=(int16_t)(x>32767.0?32767.0:x<-32768.0?-32768.0:x);
                p[k].sy=(int16_t)(y>32767.0?32767.0:y<-32768.0?-32768.0:y);
            }
        }
        for(unsigned k=0;k<st->count;++k) {
            const uint32_t roll_=random_word()%100u;
            p[k].outcode=(uint8_t)(roll_<3u?random_word():roll_<40u?(random_word()&0x3cu):0u);
            p[k].x=(int32_t)random_word();p[k].z=(int32_t)random_word();   /* never read */
        }
    }
}
static uint32_t random_mask(void) {
    switch(random_word()%10u) {
    case 0: case 1: case 2: case 3: return 0x16u;          /* runs of 1, 2, 4 */
    case 4: return 0x06u;
    case 5: return 0x12u;
    case 6: return 0x116u;
    case 7: return 0xfffffffeu;
    default: return random_word();
    }
}
static const uint32_t *random_ends(strip_t *st) {
    switch(random_word()%10u) {
    case 0: case 1: case 2: case 3: case 4: return NULL;
    case 5: case 6: case 7: {   /* tier textures: a run of r starts and ends on multiples of r from a phase */
        const unsigned phase=random_word()%8u;
        for(unsigned j=0;j<st->count;++j) {
            st->ends[j]=0;
            for(unsigned r=2;r<32u;++r)if((j+phase)%r==0u)st->ends[j]|=UINT32_C(1)<<r;
        }
        return st->ends;
    }
    default:
        for(unsigned j=0;j<st->count;++j)st->ends[j]=random_word()%4u?random_word():0u;
        return st->ends;
    }
}
static uint32_t random_tol(unsigned mode) {
    static const uint32_t pixels[]={256,256,128,512,64,1024,0,3};
    static const uint32_t rows[]={256,256,128,512,64,2048,0,3};
    const uint32_t r=random_word()%10u;
    if(r<8u)return mode==SH2Z_VIEW_MERGE_PIXELS?pixels[r]:rows[r];
    return r==8u?SH2Z_VIEW_MERGE_MAX_TOL-(random_word()%2u):random_word()%4096u;
}

/* ---- the documented bound, in floating point ------------------------------ */
typedef struct {double x,y;} pt_t;
static pt_t ptv(const sh2z_view_vertex_t *a) {pt_t p={a->sx,a->sy};return p;}
static pt_t lerp(pt_t a,pt_t b,double f) {pt_t p={a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f};return p;}
static double cheb(pt_t a,pt_t b) {const double dx=fabs(a.x-b.x),dy=fabs(a.y-b.y);return dx>dy?dx:dy;}
static double cross2(pt_t a,pt_t b) {return a.x*b.y-a.y*b.x;}
static pt_t sub(pt_t a,pt_t b) {pt_t p={a.x-b.x,a.y-b.y};return p;}

/* Largest boundary displacement of a run on both rails (pixels), and in rows
 * mode the largest along-rail row error. */
static void run_errors(const strip_t *st,unsigned i,unsigned r,double *pix,double *rows_err) {
    *pix=0;*rows_err=0;
    const double s=(double)st->rows[i+r]-st->rows[i];
    for(unsigned side=0;side<2u;++side) {
        const sh2z_view_vertex_t *p=side?st->right:st->left;
        const pt_t p0=ptv(&p[i]),pn=ptv(&p[i+r]),d=sub(pn,p0);
        const double d2=d.x*d.x+d.y*d.y;
        for(unsigned k=1;k<r;++k) {
            const double t=(double)st->rows[i+k]-st->rows[i];
            const pt_t pk=ptv(&p[i+k]);
            const double e=cheb(pk,lerp(p0,pn,t/s));
            if(e>*pix)*pix=e;
            if(d2>0) {
                const pt_t a=sub(pk,p0);
                const double re=fabs((a.x*d.x+a.y*d.y)/d2*s-t);
                if(re>*rows_err)*rows_err=re;
            }
        }
    }
}

/* Inverse of the bilinear quad a (0,0), b (1,0), c (1,1), d (0,1): u runs
 * a -> b and w runs a -> d. */
static bool inv_bilinear(pt_t p,pt_t a,pt_t b,pt_t c,pt_t d,double *u,double *w) {
    const pt_t e=sub(b,a),f=sub(d,a),h=sub(p,a);
    const pt_t g={a.x-b.x+c.x-d.x,a.y-b.y+c.y-d.y};
    const double k2=cross2(g,f),k1=cross2(e,f)+cross2(h,g),k0=cross2(h,e);
    double roots[2];
    unsigned nr=0;
    const double scale=fabs(k1)+fabs(k0)+fabs(k2)+1e-300;
    if(fabs(k2)<1e-12*scale) {
        if(fabs(k1)<1e-300)return false;
        roots[nr++]=-k0/k1;
    } else {
        double disc=k1*k1-4.0*k2*k0;
        if(disc<0) {if(disc<-1e-9*k1*k1)return false;disc=0;}
        const double q=-0.5*(k1+(k1<0?-sqrt(disc):sqrt(disc)));
        roots[nr++]=q/k2;
        if(fabs(q)>1e-300)roots[nr++]=k0/q;
    }
    for(unsigned m=0;m<nr;++m) {
        const double ww=roots[m];
        if(ww<-1e-9 || ww>1+1e-9)continue;
        const pt_t pa=lerp(a,d,ww),pb=lerp(b,c,ww),seg=sub(pb,pa);
        const double l2=seg.x*seg.x+seg.y*seg.y;
        if(l2<=0)continue;
        const pt_t q=sub(p,pa);
        const double uu=(q.x*seg.x+q.y*seg.y)/l2;
        if(uu<-1e-9 || uu>1+1e-9)continue;
        if(cheb(lerp(pa,pb,uu),p)>1e-6)continue;
        *u=uu;*w=ww;
        return true;
    }
    return false;
}
static bool convex_quad(pt_t a,pt_t b,pt_t c,pt_t d) {
    const pt_t q[4]={a,b,c,d};
    double sign=0;
    for(unsigned k=0;k<4u;++k) {
        const double z=cross2(sub(q[(k+1)&3u],q[k]),sub(q[(k+2)&3u],q[(k+1)&3u]));
        if(fabs(z)<0.5)return false;
        if(sign==0)sign=z;
        else if((z>0)!=(sign>0))return false;
    }
    return true;
}

typedef struct {
    uint64_t runs,samples,inverse_runs,inverse_samples,uncovered,degenerate,inv_over;
    double fwd_excess,fwd_vs_boundary,max_fwd_ratio;          /* forward: texel -> screen */
    double inv_max_rail,inv_max_interior,inv_excess_straight,inv_excess_curved,inv_max_ratio,inv_local_px;
} property_t;

/* Sample one merged quad of the plan against the per-repetition reference.
 * Forward: every texel (u, v), its merged and reference screen positions;
 * the claim is that the largest Chebyshev distance is the largest at the
 * boundary vertices. Inverse (rows mode): every merged sample's screen point
 * mapped back through the reference; the row error there. */
static void property_run(property_t *pr,const strip_t *st,unsigned i,unsigned r,uint32_t tol,unsigned mode) {
    enum { Q=12, U=8 };
    double bpix,brows;
    run_errors(st,i,r,&bpix,&brows);
    const double s=(double)st->rows[i+r]-st->rows[i];
    const pt_t l0=ptv(&st->left[i]),ln=ptv(&st->left[i+r]),r0=ptv(&st->right[i]),rn=ptv(&st->right[i+r]);
    bool invertible=convex_quad(l0,r0,rn,ln);
    for(unsigned j=0;j<r && invertible;++j)
        invertible=convex_quad(ptv(&st->left[i+j]),ptv(&st->right[i+j]),ptv(&st->right[i+j+1]),ptv(&st->left[i+j+1]));
    double fwd=0,inv_rail=0,inv_int=0;
    for(unsigned j=0;j<r;++j) {
        const double tj=(double)st->rows[i+j]-st->rows[i],tn=(double)st->rows[i+j+1]-st->rows[i];
        const pt_t lj=ptv(&st->left[i+j]),lj1=ptv(&st->left[i+j+1]),rj=ptv(&st->right[i+j]),rj1=ptv(&st->right[i+j+1]);
        for(unsigned q=0;q<=Q;++q) {
            const double f=(double)q/Q,v=tj+(tn-tj)*f;
            const pt_t lm=lerp(l0,ln,v/s),rm=lerp(r0,rn,v/s),lr=lerp(lj,lj1,f),rr=lerp(rj,rj1,f);
            for(unsigned uq=0;uq<=U;++uq) {
                const double u=(double)uq/U;
                const pt_t xm=lerp(lm,rm,u),xr=lerp(lr,rr,u);
                const double e=cheb(xm,xr);
                if(e>fwd)fwd=e;
                ++pr->samples;
                if(!invertible || mode!=SH2Z_VIEW_MERGE_ROWS)continue;
                /* Which texture row does the reference show at xm? */
                bool found=false;
                for(unsigned m=0;m<r && !found;++m) {
                    double uu,ww;
                    if(inv_bilinear(xm,ptv(&st->left[i+m]),ptv(&st->right[i+m]),ptv(&st->right[i+m+1]),ptv(&st->left[i+m+1]),&uu,&ww)) {
                        const double tm=(double)st->rows[i+m]-st->rows[i],tm1=(double)st->rows[i+m+1]-st->rows[i];
                        const double err=fabs(tm+(tm1-tm)*ww-v);
                        /* The same excess in pixels at the reference's own row
                         * density there: rows per pixel along its sub-quad. */
                        const double el=hypot(st->left[i+m+1].sx-st->left[i+m].sx,st->left[i+m+1].sy-st->left[i+m].sy);
                        const double er=hypot(st->right[i+m+1].sx-st->right[i+m].sx,st->right[i+m+1].sy-st->right[i+m].sy);
                        const double len=(1-uu)*el+uu*er;
                        if(len>0 && err>tol/256.0) {
                            const double px=(err-tol/256.0)*len/(tm1-tm);
                            if(px>pr->inv_local_px)pr->inv_local_px=px;
                        }
                        if(uq==0u || uq==U){if(err>inv_rail)inv_rail=err;}
                        else if(err>inv_int)inv_int=err;
                        found=true;
                    }
                }
                ++pr->inverse_samples;
                pr->uncovered+=!found;
            }
        }
    }
    ++pr->runs;
    const double limit=mode==SH2Z_VIEW_MERGE_PIXELS?tol/256.0:1.0;
    check(fwd<=bpix+1e-9*(1.0+bpix),"forward error is largest at a boundary vertex");
    check(fwd>=bpix-1e-9*(1.0+bpix),"the boundary maximum is attained");
    check(bpix<=limit+1e-12,"boundary displacement within the tolerance");
    if(fwd-limit>pr->fwd_excess)pr->fwd_excess=fwd-limit;
    if(fwd-bpix>pr->fwd_vs_boundary)pr->fwd_vs_boundary=fwd-bpix;
    if(limit>0 && fwd/limit>pr->max_fwd_ratio)pr->max_fwd_ratio=fwd/limit;
    if(mode==SH2Z_VIEW_MERGE_ROWS) {
        const double rows_limit=tol/256.0;
        check(brows<=rows_limit+1e-9*(1.0+rows_limit),"along-rail row error within the tolerance");
        if(!invertible){++pr->degenerate;return;}
        ++pr->inverse_runs;
        if(inv_rail>pr->inv_max_rail)pr->inv_max_rail=inv_rail;
        if(inv_int>pr->inv_max_interior)pr->inv_max_interior=inv_int;
        /* The excess over the tolerance in pixels, at the merged quad's
         * denser rail (S / |D| rows per pixel). */
        const double worst=inv_rail>inv_int?inv_rail:inv_int,excess=worst-rows_limit;
        const double dl=hypot(ln.x-l0.x,ln.y-l0.y),dr=hypot(rn.x-r0.x,rn.y-r0.y);
        const double dmin=dl<dr?dl:dr,excess_px=dmin>0?excess*dmin/s:0;
        pr->inv_over+=excess>1e-6;
        if(st->straight){if(excess_px>pr->inv_excess_straight)pr->inv_excess_straight=excess_px;}
        else if(excess_px>pr->inv_excess_curved)pr->inv_excess_curved=excess_px;
        if(rows_limit>0 && worst/rows_limit>pr->inv_max_ratio)pr->inv_max_ratio=worst/rows_limit;
    }
}

/* ---- sweep ---------------------------------------------------------------- */
static void plan_properties(const strip_t *st,const uint8_t *runs,int n,uint32_t mask,const uint32_t *ends,uint32_t tol,unsigned mode) {
    unsigned i=0;
    for(int k=0;k<n;++k) {
        const unsigned r=runs[k];
        check(r>=1u && r<=31u && i+r<=st->count-1u,"run length in range");
        if(r>1u) {
            check(((mask>>r)&1u)!=0,"run allowed by the mask");
            check(!ends || ((ends[i+r]>>r)&1u),"run allowed by its end mask");
            for(unsigned j=i;j<=i+r;++j)check(!((st->left[j].outcode|st->right[j].outcode)&BLOCKING),"no blocked boundary in a merged run");
            double pix,rows_err;
            run_errors(st,i,r,&pix,&rows_err);
            if(mode==SH2Z_VIEW_MERGE_PIXELS)check(pix<=tol/256.0*(1.0+1e-12)+1e-12,"pixel bound holds");
            else {
                check(pix<=1.0+1e-12,"rows mode pixel guard holds");
                check(rows_err<=tol/256.0*(1.0+1e-12)+1e-9,"rows bound holds");
            }
        }
        i+=r;
    }
    check(i==st->count-1u,"runs cover every repetition");
}
typedef struct { uint64_t strips,reps,runs,merged[32]; } usage_t;

static void sweep(void) {
    enum { ROAD=60000, ADVERSARIAL=60000, PROPERTY_STRIPS=6000 };
    static strip_t st;
    uint64_t compared=0,merged_runs=0,boundaries=0,blocked_runs=0,huge=0;
    uint64_t control[5]={0};
    static const unsigned controls[5]={MUT_INTERIOR,MUT_NO_RIGHT,MUT_NO_GUARD,MUT_SMALLEST,MUT_ENDS};
    static const char *const names[5]={"interior loop stops one boundary early","right rail not checked",
        "rows mode without the pixel guard","greedy takes the smallest passing run","NEAR/FAR checked only inside the run"};
    property_t prop[2];memset(prop,0,sizeof(prop));
    usage_t usage[2];memset(usage,0,sizeof(usage));
    for(unsigned trial=0;trial<ROAD+ADVERSARIAL;++trial) {
        const bool road=trial<ROAD;
        if(road)road_strip(&st);else adversarial_strip(&st);
        const uint32_t mask=random_mask();
        const uint32_t *ends=random_ends(&st);
        const unsigned mode=random_word()%2u;
        const uint32_t tol=random_tol(mode);
        if((uint64_t)st.rows[st.count-1]-st.rows[0]>UINT64_C(0x80000000))++huge;
        uint8_t got[MAXB+8],want[MAXB+8],sentinel[MAXB+8];
        memset(got,0xa5,sizeof(got));memcpy(sentinel,got,sizeof(got));
        const int n=sh2z_view_merge_runs(got,st.left,st.right,st.rows,ends,st.count,mask,tol,mode);
        const int w=oracle_plan(want,&st,mask,ends,tol,mode,MUT_NONE);
        if(n!=w || memcmp(got,want,(size_t)n)) {
            fprintf(stderr,"trial %u road %d count %u mask %08x ends %d tol %u mode %u n %d w %d\n",trial,(int)road,st.count,mask,ends!=NULL,tol,mode,n,w);
            for(int k=0;k<n||k<w;++k)fprintf(stderr,"  %d: got %d want %d\n",k,k<n?got[k]:-1,k<w?want[k]:-1);
            for(unsigned k=0;k<st.count;++k)fprintf(stderr,"  row %u: %u\n",k,st.rows[k]);
        }
        check(n==w,"run count equals the oracle");
        check(!memcmp(got,want,(size_t)n),"runs equal the oracle");
        check(!memcmp(got+n,sentinel+n,sizeof(got)-(size_t)n),"nothing written past the runs");
        compared+=(uint64_t)n;boundaries+=st.count;
        plan_properties(&st,got,n,mask,ends,tol,mode);
        for(int k=0;k<n;++k)merged_runs+=got[k]>1u;
        for(unsigned c=0;c<5u;++c) {
            uint8_t m[MAXB+8];
            const int nm=oracle_plan(m,&st,mask,ends,tol,mode,controls[c]);
            if(nm!=n || memcmp(m,got,(size_t)n))++control[c];
        }
        /* A blocked boundary forced a shorter plan than the same strip without flags. */
        if(!road) {
            static strip_t clean;
            clean=st;
            for(unsigned j=0;j<st.count;++j){clean.left[j].outcode&=0x3cu;clean.right[j].outcode&=0x3cu;}
            uint8_t m[MAXB+8];
            blocked_runs+=oracle_plan(m,&clean,mask,ends,tol,mode,MUT_NONE)!=n;
        }
        if(road && trial<PROPERTY_STRIPS*4u) {
            unsigned i=0;
            for(int k=0;k<n;++k){if(got[k]>1u && prop[mode].runs<(uint64_t)PROPERTY_STRIPS*4u)property_run(&prop[mode],&st,i,got[k],tol,mode);i+=got[k];}
        }
        /* The caller's case: 64-boundary road strips, runs of 1, 2, 4, no end masks, 1 px / 1 row. */
        if(road && st.count==64u) {
            for(unsigned md=0;md<2u;++md) {
                uint8_t m[MAXB+8],o[MAXB+8];
                const int nm=sh2z_view_merge_runs(m,st.left,st.right,st.rows,NULL,64,0x16u,256,md);
                check(nm==oracle_plan(o,&st,0x16u,NULL,256,md,MUT_NONE) && !memcmp(m,o,(size_t)nm),"usage plan");
                usage[md].strips++;usage[md].reps+=63;usage[md].runs+=(uint64_t)nm;
                for(int k=0;k<nm;++k)usage[md].merged[m[k]]++;
            }
        }
    }
    printf("sweep: %u road strips + %u adversarial strips (%llu with S > 2^31), %llu boundaries; %llu runs equal the oracle (%llu merged)\n",
        ROAD,ADVERSARIAL,(unsigned long long)huge,(unsigned long long)boundaries,(unsigned long long)compared,(unsigned long long)merged_runs);
    printf("  %llu adversarial strips planned differently because of NEAR/FAR/undefined flags\n",(unsigned long long)blocked_runs);
    for(unsigned c=0;c<5u;++c)printf("control '%s': %llu plans differ\n",names[c],(unsigned long long)control[c]);
    for(unsigned md=0;md<2u;++md)
        printf("usage, %s mode, tol 256, runs {1,2,4}, 64 boundaries: %llu strips, %.2f quads per 63 repetitions (x1 %llu, x2 %llu, x4 %llu)\n",
            md?"rows":"pixel",(unsigned long long)usage[md].strips,(double)usage[md].runs/(double)usage[md].strips,
            (unsigned long long)usage[md].merged[1],(unsigned long long)usage[md].merged[2],(unsigned long long)usage[md].merged[4]);
    const property_t *pp=&prop[0],*pr=&prop[1];
    printf("property, pixel mode: %llu merged quads, %llu texel samples; forward error max %.3g x tol, never above the boundary maximum (+%.3g px)\n",
        (unsigned long long)pp->runs,(unsigned long long)pp->samples,pp->max_fwd_ratio,pp->fwd_vs_boundary);
    printf("property, rows mode: %llu merged quads (%llu invertible, %llu degenerate), %llu samples; forward max %.3g px over the 1 px guard bound (+%.3g over boundary max)\n",
        (unsigned long long)pr->runs,(unsigned long long)pr->inverse_runs,(unsigned long long)pr->degenerate,(unsigned long long)pr->inverse_samples,
        pr->fwd_excess,pr->fwd_vs_boundary);
    printf("  row error at screen points: rails max %.4g rows, interior max %.4g rows (worst %.3g x tol); %llu quads exceed tol, by at most %.3f px (straight rails) / %.3f px (curved) at the denser rail, %.3f px at the reference's local density; %llu samples outside the reference\n",
        pr->inv_max_rail,pr->inv_max_interior,pr->inv_max_ratio,(unsigned long long)pr->inv_over,pr->inv_excess_straight,pr->inv_excess_curved,pr->inv_local_px,(unsigned long long)pr->uncovered);
    check(merged_runs>100000u && huge>1000u && blocked_runs>1000u,"sweep coverage");
    for(unsigned c=0;c<5u;++c)check(control[c]>0,"every control is detected");
    check(pp->runs>10000u && pr->runs>10000u && pr->inverse_runs>5000u,"property coverage");
    check(pp->fwd_vs_boundary<=1e-9 && pr->fwd_vs_boundary<=1e-9,"forward error never exceeds its boundary maximum");
}

/* ---- hand cases ----------------------------------------------------------- */
static void set_vertex(sh2z_view_vertex_t *p,int32_t x,int32_t y) {memset(p,0,sizeof(*p));p->sx=(int16_t)x;p->sy=(int16_t)y;}
static void hand_cases(void) {
    static strip_t st;
    uint8_t runs[MAXB];
    /* Linear on screen: every merge is exact, even at tolerance 0. */
    st.count=64;
    for(unsigned k=0;k<64u;++k){set_vertex(&st.left[k],100,(int32_t)(10+8*k));set_vertex(&st.right[k],200,(int32_t)(10+8*k));st.rows[k]=1000u+16u*k;}
    for(unsigned md=0;md<2u;++md) {
        int n=sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,64,0xfffffffeu,0,md);
        check(n==3 && runs[0]==31 && runs[1]==31 && runs[2]==1,"linear strip: 31 + 31 + 1");
        n=sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,64,0x16u,0,md);
        check(n==17 && runs[14]==4 && runs[15]==2 && runs[16]==1,"linear strip, runs {1,2,4}: 15 x 4, 2, 1");
        n=sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,64,0,0,md);
        check(n==63,"empty mask: singles only");
    }
    /* A NEAR boundary at 9 and a FAR one at 20: no merged run includes them. */
    st.left[9].outcode=SH2Z_OUT_NEAR;st.right[20].outcode=SH2Z_OUT_FAR;
    int n=sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,64,0x16u,0,SH2Z_VIEW_MERGE_PIXELS);
    {
        static const uint8_t want[]={4,4,1,1,4,4,1,1,1,4,4,4,4,4,4,4,4,4,4,2};   /* 0-8 | 9 | 10-18, 19 | 20 | 21-63 */
        check(n==(int)sizeof(want) && !memcmp(runs,want,sizeof(want)),"blocked boundaries are never inside or at the end of a merged run");
    }
    st.left[9].outcode=SH2Z_OUT_LEFT|SH2Z_OUT_TOP;st.right[20].outcode=SH2Z_OUT_BOTTOM;   /* side bits do not block */
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,64,0x16u,0,SH2Z_VIEW_MERGE_PIXELS)==17,"side bits do not block");
    st.left[9].outcode=0x40;
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,64,0x16u,0,SH2Z_VIEW_MERGE_PIXELS)==18,"undefined bits block");
    st.left[9].outcode=0;st.right[20].outcode=0;
    /* End masks: runs of 4 only on multiples of 4 from 2, runs of 2 on even boundaries. */
    for(unsigned j=0;j<64u;++j)st.ends[j]=((j%2u==0u)?4u:0u)|(((j+2u)%4u==0u)?16u:0u);
    n=sh2z_view_merge_runs(runs,st.left,st.right,st.rows,st.ends,64,0x16u,0,SH2Z_VIEW_MERGE_PIXELS);
    check(n==17 && runs[0]==2 && runs[1]==4 && runs[15]==4 && runs[16]==1,"end masks align the runs");
    /* Rows mode: a rail bowed 2 px sideways while rows advance evenly along
     * it passes the along-rail test and fails only the 1 px guard. */
    st.count=5;
    for(unsigned k=0;k<5u;++k){set_vertex(&st.left[k],k==2u?102:100,(int32_t)(10*k));set_vertex(&st.right[k],200,(int32_t)(10*k));st.rows[k]=8u*k;}
    n=sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,5,0x16u,256,SH2Z_VIEW_MERGE_ROWS);
    check(n==2 && runs[0]==2 && runs[1]==2,"rows mode: the bowed boundary stops the run of 4");
    uint8_t m[8];
    check(oracle_plan(m,&st,0x16u,NULL,256,SH2Z_VIEW_MERGE_ROWS,MUT_NO_GUARD)==1 && m[0]==4,"without the guard it would merge");
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,5,0x16u,512,SH2Z_VIEW_MERGE_PIXELS)==1,"pixel mode at 2 px merges it");
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,5,0x16u,511,SH2Z_VIEW_MERGE_PIXELS)==2,"pixel mode just under 2 px does not");
    /* Rows mode measures along the rail: a 10-pixel rail of 80 rows, middle
     * boundary 1 px late is 8 rows off. */
    for(unsigned k=0;k<3u;++k){set_vertex(&st.left[k],0,(int32_t)(5*k+(k==1u)));set_vertex(&st.right[k],50,(int32_t)(5*k));st.rows[k]=40u*k;}
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,8*256,SH2Z_VIEW_MERGE_ROWS)==1,"8 rows off passes at 8 rows");
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,8*256-1,SH2Z_VIEW_MERGE_ROWS)==2,"and fails just under");
    /* Coinciding rail ends: the row half passes, the guard still applies. */
    for(unsigned k=0;k<3u;++k){set_vertex(&st.left[k],7,k==1u?8:7);set_vertex(&st.right[k],50,(int32_t)(5*k));st.rows[k]=10u*k;}
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,0,SH2Z_VIEW_MERGE_ROWS)==1,"zero-length rail within 1 px merges");
    st.left[1].sy=9;
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,SH2Z_VIEW_MERGE_MAX_TOL,SH2Z_VIEW_MERGE_ROWS)==2,"zero-length rail beyond 1 px does not");
    /* Extremes: int16 corners and S = 2^32 - 1, exact at the threshold. */
    st.count=3;
    set_vertex(&st.left[0],INT16_MIN,INT16_MIN);set_vertex(&st.left[1],0,0);set_vertex(&st.left[2],INT16_MAX,INT16_MAX);
    st.right[0]=st.left[0];st.right[1]=st.left[1];st.right[2]=st.left[2];
    st.rows[0]=0;st.rows[1]=0x80000000u;st.rows[2]=0xffffffffu;
    /* E = S * 32768 - 2^31 * 65535 = 2^31 - 32768 per axis: the boundary is
     * 0.49999... px away; tol 127 fails and 128 passes. */
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,128,SH2Z_VIEW_MERGE_PIXELS)==1,"extreme case passes at 0.5 px");
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,127,SH2Z_VIEW_MERGE_PIXELS)==2,"extreme case fails just under");
    /* In rows: E . D / |D|^2 = (2^31 - 2^15) / 65535 = 32768 rows exactly. */
    check(sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,32768u*256u,SH2Z_VIEW_MERGE_ROWS)==1 &&
          sh2z_view_merge_runs(runs,st.left,st.right,st.rows,NULL,3,0x6u,32768u*256u-1u,SH2Z_VIEW_MERGE_ROWS)==2,"extreme case in rows mode");
    printf("hand cases: linear strips, blocking flags, end masks, the rows-mode guard, thresholds and int16/2^32 extremes\n");
}

static void argument_rules(void) {
    enum { N=8 };
    sh2z_view_vertex_t l[N],r[N];
    uint32_t rows[N],ends[N];
    memset(l,0,sizeof(l));memset(r,0,sizeof(r));
    for(unsigned k=0;k<N;++k){rows[k]=4u*k;ends[k]=~0u;l[k].sy=(int16_t)k;r[k].sy=(int16_t)k;}
    uint8_t out[N],sentinel[N];
    memset(out,0x5a,sizeof(out));memcpy(sentinel,out,sizeof(out));
    const int bad=SH2Z_VIEW_MERGE_INVALID;
    check(sh2z_view_merge_runs(NULL,l,r,rows,NULL,N,0x16u,256,0)==bad,"null out");
    check(sh2z_view_merge_runs(out,NULL,r,rows,NULL,N,0x16u,256,0)==bad,"null left");
    check(sh2z_view_merge_runs(out,l,NULL,rows,NULL,N,0x16u,256,0)==bad,"null right");
    check(sh2z_view_merge_runs(out,l,r,NULL,NULL,N,0x16u,256,0)==bad,"null rows");
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,0,0x16u,256,0)==bad && sh2z_view_merge_runs(out,l,r,rows,NULL,1,0x16u,256,0)==bad,"count >= 2");
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,SH2Z_VIEW_MERGE_MAX_COUNT+1u,0x16u,256,0)==bad,"count <= max");
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,N,0x16u,256,2)==bad,"mode");
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,N,0x16u,SH2Z_VIEW_MERGE_MAX_TOL+1u,0)==bad &&
          sh2z_view_merge_runs(out,l,r,rows,NULL,N,0x16u,SH2Z_VIEW_MERGE_MAX_TOL+1u,1)==bad,"tolerance <= max");
    rows[5]=rows[4];
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,N,0x16u,256,0)==bad,"rows equal");
    rows[5]=rows[4]-1u;
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,N,0x16u,256,1)==bad,"rows decreasing");
    rows[5]=20;rows[6]=0xfffffff0u;rows[7]=5;   /* the last step wraps */
    check(sh2z_view_merge_runs(out,l,r,rows,ends,N,0x16u,256,0)==bad,"rows wrap");
    rows[6]=24;rows[7]=28;
    check(!memcmp(out,sentinel,sizeof(out)),"rejected calls write nothing");
    /* Overlap with any input array. */
    static _Alignas(8) uint8_t arena[4096];
    sh2z_view_vertex_t *al=(sh2z_view_vertex_t *)(void *)arena;
    uint32_t *ar=(uint32_t *)(void *)(arena+1024);
    memcpy(al,l,sizeof(l));memcpy(ar,rows,sizeof(rows));memcpy(arena+2048,ends,sizeof(ends));
    memcpy(arena+3072,r,sizeof(r));
    sh2z_view_vertex_t *arr=(sh2z_view_vertex_t *)(void *)(arena+3072);
    uint32_t *ae=(uint32_t *)(void *)(arena+2048);
    uint8_t before[4096];memcpy(before,arena,sizeof(arena));
    check(sh2z_view_merge_runs(arena+sizeof(l)-1,al,arr,ar,NULL,N,0x16u,256,0)==bad,"out overlaps left");
    check(sh2z_view_merge_runs(arena+3072+sizeof(r)-1,al,arr,ar,NULL,N,0x16u,256,0)==bad,"out overlaps right");
    check(sh2z_view_merge_runs(arena+1024+sizeof(rows)-1,al,arr,ar,NULL,N,0x16u,256,0)==bad,"out overlaps rows");
    check(sh2z_view_merge_runs(arena+2048+4,al,arr,ar,ae,N,0x16u,256,0)==bad,"out overlaps end masks");
    check(!memcmp(before,arena,sizeof(arena)),"overlapping calls write nothing");
    check(sh2z_view_merge_runs(arena+1024+sizeof(rows),al,arr,ar,ae,N,0x16u,256,0)>0,"adjacent buffers are fine");
    check(sh2z_view_merge_runs(arena+1024-(N-1),al,arr,ar,ae,N,0x16u,256,0)>0,"out ending just before rows is fine");
    /* Smallest and largest counts; end_masks NULL accepted; bits 0 and 1 ignored. */
    check(sh2z_view_merge_runs(out,l,r,rows,NULL,2,0xffffffffu,0,0)==1 && out[0]==1,"count 2: one run");
    static sh2z_view_vertex_t bl[SH2Z_VIEW_MERGE_MAX_COUNT],br[SH2Z_VIEW_MERGE_MAX_COUNT];
    static uint32_t brows[SH2Z_VIEW_MERGE_MAX_COUNT];
    static uint8_t bout[SH2Z_VIEW_MERGE_MAX_COUNT];
    for(unsigned k=0;k<SH2Z_VIEW_MERGE_MAX_COUNT;++k){bl[k].sx=(int16_t)(k/4u);br[k].sx=(int16_t)(k/4u+50u);brows[k]=k*65537u;}
    const int nb=sh2z_view_merge_runs(bout,bl,br,brows,NULL,SH2Z_VIEW_MERGE_MAX_COUNT,0x16u,64,0);
    unsigned sum=0;for(int k=0;k<nb;++k)sum+=bout[k];
    check(nb>0 && sum==SH2Z_VIEW_MERGE_MAX_COUNT-1u,"largest count");
    printf("argument rules: null pointers, count, mode, tolerance, row order and wrap, overlap; nothing written on rejection\n");
}

int main(void) {
    argument_rules();
    hand_cases();
    sweep();
    printf("merge checks: %llu passed\n",(unsigned long long)checks);
    return EXIT_SUCCESS;
}
