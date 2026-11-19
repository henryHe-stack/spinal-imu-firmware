/* Independent rotation-matrix fixtures. No production quaternion helper is
 * used to construct expected rotations. Host test only; no HAL/RTOS/Python. */
#include "neck_relative.h"
#include "neck_simulator.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,groups;
static double max_matrix_error;
static const double radians=0.01745329251994329577;
static const double identity_mounts[3][4]={{1,0,0,0},{1,0,0,0},{1,0,0,0}};
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
typedef struct { double v[9]; } Matrix;
static Matrix matrix_euler(double pitch, double roll, double yaw)
{
    double cp=cos(pitch*radians),sp=sin(pitch*radians);
    double cr=cos(roll*radians),sr=sin(roll*radians);
    double cy=cos(yaw*radians),sy=sin(yaw*radians);
    Matrix m={{cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
        sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, -sp,cp*sr,cp*cr}};
    return m;
}
static Matrix product(Matrix a, Matrix b)
{
    Matrix m={{0}}; unsigned r,c,k;
    for(r=0;r<3;++r) for(c=0;c<3;++c) for(k=0;k<3;++k)
        m.v[r*3+c]+=a.v[r*3+k]*b.v[k*3+c];
    return m;
}
static Matrix transpose(Matrix a)
{ Matrix m; unsigned r,c; for(r=0;r<3;++r) for(c=0;c<3;++c) m.v[r*3+c]=a.v[c*3+r]; return m; }
static void matrix_to_quaternion(Matrix m, double q[4])
{
    double trace=m.v[0]+m.v[4]+m.v[8], s;
    if(trace>0) {
        s=sqrt(trace+1)*2; q[0]=s/4; q[1]=(m.v[7]-m.v[5])/s;
        q[2]=(m.v[2]-m.v[6])/s; q[3]=(m.v[3]-m.v[1])/s;
    } else if(m.v[0]>m.v[4] && m.v[0]>m.v[8]) {
        s=sqrt(1+m.v[0]-m.v[4]-m.v[8])*2; q[0]=(m.v[7]-m.v[5])/s;
        q[1]=s/4; q[2]=(m.v[1]+m.v[3])/s; q[3]=(m.v[2]+m.v[6])/s;
    } else if(m.v[4]>m.v[8]) {
        s=sqrt(1+m.v[4]-m.v[0]-m.v[8])*2; q[0]=(m.v[2]-m.v[6])/s;
        q[1]=(m.v[1]+m.v[3])/s; q[2]=s/4; q[3]=(m.v[5]+m.v[7])/s;
    } else {
        s=sqrt(1+m.v[8]-m.v[0]-m.v[4])*2; q[0]=(m.v[3]-m.v[1])/s;
        q[1]=(m.v[2]+m.v[6])/s; q[2]=(m.v[5]+m.v[7])/s; q[3]=s/4;
    }
}
static Matrix quaternion_matrix(const double q[4])
{
    double w=q[0],x=q[1],y=q[2],z=q[3];
    Matrix m={{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
        2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
        2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}};
    return m;
}
static void near(double a, double b)
{ CHECK(isfinite(a)); CHECK(fabs(a-b)<1e-9); }
static void compare_matrix(Matrix a, Matrix b)
{
    unsigned i; for(i=0;i<9;++i) {
        double error=fabs(a.v[i]-b.v[i]);
        if(error>max_matrix_error) max_matrix_error=error;
        near(a.v[i],b.v[i]);
    }
}
static Matrix expected_pair(const Matrix body[3], unsigned pair)
{
    static const unsigned ref[4]={2,1,2,3},target[4]={0,0,1,2};
    return ref[pair]==3?body[target[pair]]:product(transpose(body[ref[pair]]),body[target[pair]]);
}
static NeckOrientationFrame frame(uint64_t sequence, uint64_t time_us)
{
    NeckOrientationFrame f; unsigned i;
    memset(&f,0,sizeof(f)); f.config_id=f.session_id=1; f.sequence=sequence;
    f.frame_time_us=time_us; f.source=NECK_SOURCE_SIMULATION;
    for(i=0;i<3;++i) {
        f.nodes[i].sensor_to_world_wxyz[0]=1;
        f.nodes[i].sample_time_us=time_us; f.nodes[i].world_frame_id=1;
        f.nodes[i].quat_present=f.nodes[i].sample_time_known=1;
        f.nodes[i].calibration_system=3; f.nodes[i].status=NECK_NODE_OK;
    }
    return f;
}
static void fresh(NeckRelativeContext *c)
{
    NeckRelativeParameters p; neck_relative_default_parameters(&p);
    CHECK(neck_relative_init(c,1,1,identity_mounts,&p)==NECK_OK);
}
static NeckRelativeOutput process(NeckRelativeContext *c, NeckOrientationFrame *f)
{
    NeckRelativeOutput o; memset(&o,0xa5,sizeof(o));
    CHECK(neck_relative_process(c,f,&o)==NECK_OK); return o;
}
static void available(const NeckRelativeOutput *o, unsigned pair, int yes)
{
    CHECK(!!(o->pairs[pair].valid_mask&NECK_REL_RAW_QUAT)==yes);
    CHECK(o->pairs[pair].flags&NECK_REL_QUALITY_NOT_EVALUATED);
    CHECK(o->pairs[pair].flags&NECK_REL_THRESHOLDS_UNSET);
    if(!yes) CHECK(o->pairs[pair].status==NECK_REL_UNAVAILABLE);
}
static void test_arguments(void)
{
    NeckRelativeContext c; NeckRelativeParameters p; NeckRelativeOutput o;
    NeckOrientationFrame f=frame(0,0); double mounts[3][4];
    ++groups; neck_relative_default_parameters(&p); neck_relative_default_parameters(NULL);
    CHECK(p.stale_after_us==30000 && p.max_node_skew_us==10000);
    CHECK(neck_relative_init(NULL,1,1,identity_mounts,&p)==NECK_BAD_ARGUMENT);
    CHECK(neck_relative_init(&c,0,1,identity_mounts,&p)==NECK_BAD_CONFIG);
    CHECK(neck_relative_init(&c,1,0,identity_mounts,&p)==NECK_BAD_CONFIG);
    CHECK(neck_relative_init(&c,1,1,NULL,&p)==NECK_BAD_CONFIG);
    CHECK(neck_relative_init(&c,1,1,identity_mounts,NULL)==NECK_BAD_CONFIG);
    p.gimbal_cos_min=NAN;
    CHECK(neck_relative_init(&c,1,1,identity_mounts,&p)==NECK_BAD_CONFIG);
    CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_CONFIG);
    neck_relative_default_parameters(&p); p.quaternion_norm_tolerance=1;
    CHECK(neck_relative_init(&c,1,1,identity_mounts,&p)==NECK_BAD_CONFIG);
    neck_relative_default_parameters(&p); p.stale_after_us=0;
    CHECK(neck_relative_init(&c,1,1,identity_mounts,&p)==NECK_BAD_CONFIG);
    neck_relative_default_parameters(&p); memcpy(mounts,identity_mounts,sizeof(mounts)); mounts[0][0]=0;
    CHECK(neck_relative_init(&c,1,1,(const double (*)[4])mounts,&p)==NECK_BAD_CONFIG);
    fresh(&c); CHECK(neck_relative_process(NULL,&f,&o)==NECK_BAD_ARGUMENT);
    CHECK(neck_relative_process(&c,NULL,&o)==NECK_BAD_ARGUMENT);
    CHECK(neck_relative_process(&c,&f,NULL)==NECK_BAD_ARGUMENT);
    CHECK(o.flags&NECK_REL_INPUT_REJECTED); CHECK(o.pairs[0].valid_mask==0);
    neck_relative_reset_stream(NULL,1);
}
static void test_identity_and_axes(void)
{
    unsigned axis,sign,pair; ++groups;
    for(axis=0;axis<3;++axis) for(sign=0;sign<2;++sign) {
        NeckRelativeContext c; NeckRelativeOutput o; NeckOrientationFrame f=frame(0,0);
        double angle=sign?-25:25; Matrix body[3];
        body[0]=matrix_euler(axis==0?angle:0,axis==1?angle:0,axis==2?angle:0);
        body[1]=body[2]=matrix_euler(0,0,0);
        matrix_to_quaternion(body[0],f.nodes[0].sensor_to_world_wxyz);
        fresh(&c); o=process(&c,&f);
        for(pair=0;pair<4;++pair) {
            available(&o,pair,1); CHECK(o.pairs[pair].valid_mask==71);
            CHECK(o.pairs[pair].status==NECK_REL_RAW_ONLY);
            compare_matrix(quaternion_matrix(o.pairs[pair].raw_wxyz),expected_pair(body,pair));
            near(o.pairs[pair].raw_rotation_deg,pair<2?25:0);
        }
        near(o.pairs[0].raw_euler_deg[axis],angle);
        CHECK(!c.reference_set && o.reference_id==0);
    }
}
static void test_matrix_grid_and_mounts(void)
{
    NeckRelativeContext c; NeckRelativeParameters p; Matrix mounts[3];
    double quats[3][4],saved[3][4]; unsigned n,i,pair;
    ++groups; mounts[0]=matrix_euler(20,90,-30); mounts[1]=matrix_euler(-13,7,19);
    mounts[2]=matrix_euler(-30,-5,90);
    for(i=0;i<3;++i) matrix_to_quaternion(mounts[i],quats[i]);
    memcpy(saved,quats,sizeof(saved)); neck_relative_default_parameters(&p);
    CHECK(neck_relative_init(&c,1,1,(const double (*)[4])quats,&p)==NECK_OK);
    memset(quats,0,sizeof(quats)); /* Context owns a checked mount copy. */
    for(n=0;n<240;++n) {
        NeckOrientationFrame f=frame(n,n*10000); NeckRelativeOutput o; Matrix body[3];
        for(i=0;i<3;++i) {
            body[i]=matrix_euler((int)((n*13+i*27)%150)-75,
                (int)((n*11+i*17)%280)-140,(int)((n*7+i*43)%340)-170);
            matrix_to_quaternion(product(body[i],mounts[i]),f.nodes[i].sensor_to_world_wxyz);
        }
        o=process(&c,&f);
        for(pair=0;pair<4;++pair) {
            available(&o,pair,1);
            compare_matrix(quaternion_matrix(o.pairs[pair].raw_wxyz),expected_pair(body,pair));
            if(o.pairs[pair].valid_mask&NECK_REL_RAW_EULER)
                compare_matrix(matrix_euler(o.pairs[pair].raw_euler_deg[0],o.pairs[pair].raw_euler_deg[1],
                    o.pairs[pair].raw_euler_deg[2]),expected_pair(body,pair));
        }
        compare_matrix(product(quaternion_matrix(o.pairs[2].raw_wxyz),
            quaternion_matrix(o.pairs[1].raw_wxyz)),quaternion_matrix(o.pairs[0].raw_wxyz));
    }
    CHECK(saved[0][0]!=0); /* Ensure fixture really had nonidentity mounts. */
}
static void test_common_world_rotation(void)
{
    NeckRelativeContext a,b; NeckOrientationFrame f=frame(0,0),g=frame(0,0);
    NeckRelativeOutput x,y; Matrix body[3],common=matrix_euler(33,-11,79); unsigned i,pair;
    ++groups; fresh(&a); fresh(&b);
    for(i=0;i<3;++i) {
        body[i]=matrix_euler(5+i*12,3+i*7,-4+i*9);
        matrix_to_quaternion(body[i],f.nodes[i].sensor_to_world_wxyz);
        matrix_to_quaternion(product(common,body[i]),g.nodes[i].sensor_to_world_wxyz);
    }
    x=process(&a,&f); y=process(&b,&g);
    for(pair=0;pair<3;++pair) compare_matrix(quaternion_matrix(x.pairs[pair].raw_wxyz),
        quaternion_matrix(y.pairs[pair].raw_wxyz));
    compare_matrix(quaternion_matrix(y.pairs[3].raw_wxyz),product(common,body[2]));
}
static void test_sign_and_normalization(void)
{
    NeckRelativeContext c; NeckOrientationFrame f=frame(0,0); NeckRelativeOutput a,b;
    unsigned i,k; ++groups; fresh(&c);
    for(i=0;i<3;++i) matrix_to_quaternion(matrix_euler(12+i,90-i*31,179-i*81),f.nodes[i].sensor_to_world_wxyz);
    a=process(&c,&f); f.sequence=1; f.frame_time_us=10000;
    for(i=0;i<3;++i) { f.nodes[i].sample_time_us=10000;
        for(k=0;k<4;++k) f.nodes[i].sensor_to_world_wxyz[k]*=-1.0005; }
    b=process(&c,&f);
    for(i=0;i<4;++i) {
        for(k=0;k<4;++k) near(a.pairs[i].raw_wxyz[k],b.pairs[i].raw_wxyz[k]);
        CHECK(b.pairs[i].flags&NECK_REL_NORMALIZED);
        near(hypot(hypot(b.pairs[i].raw_wxyz[0],b.pairs[i].raw_wxyz[1]),
            hypot(b.pairs[i].raw_wxyz[2],b.pairs[i].raw_wxyz[3])),1);
    }
    fresh(&c); f=frame(0,0); f.nodes[0].sensor_to_world_wxyz[0]=-1;
    f.nodes[0].sensor_to_world_wxyz[1]=-0.0; a=process(&c,&f);
    near(a.pairs[0].raw_wxyz[0],1); CHECK(!signbit(a.pairs[0].raw_wxyz[1]));
    fresh(&c); f=frame(0,0); f.nodes[0].sensor_to_world_wxyz[0]=0;
    f.nodes[0].sensor_to_world_wxyz[1]=-1; a=process(&c,&f);
    near(a.pairs[0].raw_wxyz[1],1); near(a.pairs[0].raw_rotation_deg,180);
}
static void test_invalid_quaternions(void)
{
    static const double bad[]={0,1.0011,NAN,INFINITY,DBL_MAX}; unsigned node,j,pair;
    ++groups;
    for(node=0;node<3;++node) for(j=0;j<sizeof(bad)/sizeof(bad[0]);++j) {
        NeckRelativeContext c; NeckRelativeOutput o; NeckOrientationFrame f=frame(0,0);
        fresh(&c); f.nodes[node].sensor_to_world_wxyz[0]=bad[j]; o=process(&c,&f);
        for(pair=0;pair<4;++pair) {
            int dependent=o.pairs[pair].target_node==node || o.pairs[pair].reference_node==node;
            available(&o,pair,!dependent);
            if(dependent) { CHECK(o.pairs[pair].flags&NECK_REL_INVALID_QUATERNION);
                CHECK(o.pairs[pair].valid_mask==0); near(o.pairs[pair].raw_rotation_deg,0); }
        }
    }
}
static void test_node_metadata(void)
{
    NeckRelativeContext c; NeckRelativeOutput o; NeckOrientationFrame f;
    unsigned kind,pair; ++groups;
    for(kind=0;kind<5;++kind) {
        fresh(&c); f=frame(0,0);
        if(kind==0) f.nodes[1].status=NECK_NODE_MISSING;
        if(kind==1) f.nodes[1].status=255;
        if(kind==2) f.nodes[1].quat_present=0;
        if(kind==3) f.nodes[1].world_frame_id=2;
        if(kind==4) f.nodes[1].calibration_system=4;
        o=process(&c,&f);
        for(pair=0;pair<4;++pair) available(&o,pair,pair==0||pair==3);
        CHECK(o.pairs[1].flags&(kind==3?NECK_REL_WORLD_MISMATCH:NECK_REL_NODE_UNAVAILABLE));
    }
    fresh(&c); f=frame(0,0); f.nodes[0].calibration_system=255; f.nodes[2].calibration_system=0;
    o=process(&c,&f); available(&o,0,1); CHECK(o.pairs[0].flags&NECK_REL_CALIBRATION_UNKNOWN);
    CHECK(o.pairs[0].flags&NECK_REL_CALIBRATION_PENDING);
}
static void test_timing_boundaries(void)
{
    NeckRelativeContext c; NeckRelativeOutput o; NeckOrientationFrame f;
    ++groups; fresh(&c); f=frame(0,100000);
    f.nodes[0].sample_time_us=f.nodes[1].sample_time_us=70000; f.nodes[2].sample_time_us=80000;
    o=process(&c,&f); available(&o,0,1); CHECK(o.pairs[0].sample_skew_us==10000);
    fresh(&c); f.nodes[0].sample_time_us=69999; o=process(&c,&f); available(&o,0,0);
    CHECK(o.pairs[0].flags&NECK_REL_STALE); available(&o,2,1);
    fresh(&c); f.nodes[0].sample_time_us=70000; f.nodes[2].sample_time_us=80001;
    o=process(&c,&f); available(&o,0,0); available(&o,1,1); available(&o,2,0); available(&o,3,1);
    CHECK(o.pairs[0].valid_mask==NECK_REL_KNOWN_SKEW && o.pairs[0].sample_skew_us==10001);
    CHECK(o.pairs[0].flags&NECK_REL_EXCESSIVE_SKEW);
    fresh(&c); f=frame(0,100000); f.nodes[1].sample_time_known=0;
    o=process(&c,&f); available(&o,1,0); available(&o,2,0); available(&o,0,1);
    CHECK(o.pairs[1].flags&NECK_REL_TIME_UNKNOWN);
    fresh(&c); f=frame(0,100000); f.nodes[2].sample_time_us=100001;
    o=process(&c,&f); available(&o,3,0); CHECK(o.pairs[3].flags&NECK_REL_STALE);
    fresh(&c); c.parameters.max_node_skew_us=0; f=frame(0,100000);
    f.nodes[0].sample_time_us=99999; o=process(&c,&f); available(&o,0,0); available(&o,2,1);
}
static void test_gimbal_lock(void)
{
    unsigned sign; ++groups;
    for(sign=0;sign<2;++sign) {
        NeckRelativeContext c; NeckRelativeOutput o; NeckOrientationFrame f=frame(0,0);
        Matrix m=matrix_euler(sign?-90:90,17,29);
        fresh(&c); matrix_to_quaternion(m,f.nodes[0].sensor_to_world_wxyz); o=process(&c,&f);
        CHECK((o.pairs[0].valid_mask&NECK_REL_RAW_QUAT)!=0);
        CHECK((o.pairs[0].valid_mask&NECK_REL_RAW_EULER)==0);
        CHECK(o.pairs[0].flags&NECK_REL_RAW_GIMBAL_LOCK);
        compare_matrix(quaternion_matrix(o.pairs[0].raw_wxyz),m);
        CHECK(neck_relative_capture_reference(&c,&o,1,NECK_REFERENCE_SIMULATION)==NECK_OK);
        f=frame(1,10000); o=process(&c,&f);
        CHECK((o.pairs[0].valid_mask&NECK_REL_RAW_EULER)!=0);
        compare_matrix(quaternion_matrix(o.pairs[0].corrected_wxyz),transpose(m));
        /* The inverse of a GENERAL singular pose need not be singular in
         * the same Euler chart. Pure Ry(+/-90) has a singular inverse. */
        fresh(&c); f=frame(0,0); m=matrix_euler(sign?-90:90,0,0);
        matrix_to_quaternion(m,f.nodes[0].sensor_to_world_wxyz); o=process(&c,&f);
        CHECK(neck_relative_capture_reference(&c,&o,1,NECK_REFERENCE_SIMULATION)==NECK_OK);
        f=frame(1,10000); o=process(&c,&f);
        CHECK((o.pairs[0].valid_mask&NECK_REL_CORRECTED_EULER)==0);
        CHECK(o.pairs[0].flags&NECK_REL_CORRECTED_GIMBAL_LOCK);
        CHECK((o.pairs[0].valid_mask&NECK_REL_CORRECTED_QUAT)!=0);
    }
}
static void test_reference_rotation(void)
{
    NeckRelativeContext c; NeckRelativeOutput o,old; NeckRelativeReference r;
    NeckOrientationFrame f=frame(0,0); Matrix initial[3],current[3]; unsigned i,pair;
    ++groups; fresh(&c);
    for(i=0;i<3;++i) { initial[i]=matrix_euler(10+i*12,-17+i*4,23-i*9);
        matrix_to_quaternion(initial[i],f.nodes[i].sensor_to_world_wxyz); }
    o=process(&c,&f); old=o;
    CHECK(neck_relative_capture_reference(&c,&o,7,NECK_REFERENCE_SIMULATION)==NECK_OK);
    f.sequence=1; f.frame_time_us=10000;
    for(i=0;i<3;++i) f.nodes[i].sample_time_us=10000;
    o=process(&c,&f);
    CHECK(o.reference_id==7);
    for(pair=0;pair<4;++pair) { CHECK(o.pairs[pair].valid_mask==127); near(o.pairs[pair].corrected_rotation_deg,0); }
    CHECK(neck_relative_capture_reference(&c,&old,8,NECK_REFERENCE_SIMULATION)==NECK_BAD_REFERENCE);
    r=c.reference; r.pair_wxyz[2][0]=NAN;
    CHECK(neck_relative_set_reference(&c,&r)==NECK_BAD_REFERENCE && c.reference.reference_id==7);
    CHECK(neck_relative_capture_reference(&c,&o,0,NECK_REFERENCE_SIMULATION)==NECK_BAD_REFERENCE);
    f.sequence=2; f.frame_time_us=20000;
    for(i=0;i<3;++i) { current[i]=matrix_euler(35-i*6,12+i*7,-30+i*8);
        f.nodes[i].sample_time_us=20000; matrix_to_quaternion(current[i],f.nodes[i].sensor_to_world_wxyz); }
    o=process(&c,&f);
    for(pair=0;pair<4;++pair) {
        Matrix expected=product(transpose(expected_pair(initial,pair)),expected_pair(current,pair));
        compare_matrix(quaternion_matrix(o.pairs[pair].corrected_wxyz),expected);
        compare_matrix(matrix_euler(o.pairs[pair].corrected_euler_deg[0],o.pairs[pair].corrected_euler_deg[1],
            o.pairs[pair].corrected_euler_deg[2]),expected);
    }
    f=frame(3,30000); f.nodes[0].quat_present=0; o=process(&c,&f);
    CHECK(neck_relative_capture_reference(&c,&o,8,0)==NECK_BAD_REFERENCE);
    CHECK(c.reference.reference_id==7);
    neck_relative_reset_stream(&c,0); CHECK(c.reference_set);
    neck_relative_reset_stream(&c,1); CHECK(!c.reference_set);
    CHECK(neck_relative_set_reference(NULL,&r)==NECK_BAD_ARGUMENT);
    CHECK(neck_relative_set_reference(&c,NULL)==NECK_BAD_ARGUMENT);
}
static void test_stream_provenance(void)
{
    NeckRelativeContext c; NeckRelativeOutput o; NeckOrientationFrame f=frame(0,0);
    ++groups; fresh(&c); o=process(&c,&f);
    CHECK(neck_relative_capture_reference(&c,&o,1,0)==NECK_OK);
    CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_FRAME && c.reference_set);
    CHECK(o.pairs[0].valid_mask==0 && (o.flags&NECK_REL_INPUT_REJECTED));
    f=frame(1,10000); f.config_id=2;
    CHECK(neck_relative_process(&c,&f,&o)==NECK_CONFIG_MISMATCH && c.reference_set);
    f.config_id=1; f.session_id=2;
    CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_FRAME && !c.reference_set);
    neck_relative_reset_stream(&c,1); o=process(&c,&f);
    f.sequence=2; f.frame_time_us=20000; f.source=NECK_SOURCE_HARDWARE;
    CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_FRAME);
    fresh(&c); f=frame(0,0); o=process(&c,&f);
    CHECK(neck_relative_capture_reference(&c,&o,3,0)==NECK_OK);
    neck_relative_reset_stream(&c,0); f.source=NECK_SOURCE_HARDWARE;
    CHECK(neck_relative_process(&c,&f,&o)==NECK_SIM_REFERENCE_ON_HARDWARE && !c.reference_set);
    o=process(&c,&f); /* Hardware metadata fixture, NOT actual sensor data. */
    CHECK(neck_relative_capture_reference(&c,&o,4,0)==NECK_SIM_REFERENCE_ON_HARDWARE);
    CHECK(neck_relative_capture_reference(&c,&o,4,NECK_REFERENCE_EXTERNAL)==NECK_OK);
    fresh(&c); f=frame(0,0); f.session_id=0;
    CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_FRAME);
    f.session_id=1; f.source=2; CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_FRAME);
}
static void test_large_time_and_gaps(void)
{
    uint64_t base=(UINT64_C(1)<<63)+1111; NeckRelativeContext c; NeckRelativeOutput o;
    NeckOrientationFrame f=frame(UINT64_MAX-4,base); ++groups; fresh(&c);
    o=process(&c,&f); available(&o,0,1); CHECK(o.frame_time_us==base);
    f=frame(UINT64_MAX-2,base+10000); o=process(&c,&f);
    CHECK(o.flags&NECK_REL_STREAM_GAP); available(&o,0,1);
    f=frame(UINT64_MAX-1,base+50001); o=process(&c,&f);
    CHECK(o.flags&NECK_REL_STREAM_GAP); available(&o,0,1);
    f=frame(UINT64_MAX,base+60001); o=process(&c,&f);
    CHECK(!(o.flags&NECK_REL_STREAM_GAP)); available(&o,0,1);
    f=frame(0,base+70001); CHECK(neck_relative_process(&c,&f,&o)==NECK_BAD_FRAME);
}
static void test_c_simulator_and_pipeline(const char *path)
{
    NeckRelativeContext c; NeckRelativeParameters p; NeckPipelineParameters pp;
    NeckPipelineContext pipelines[3]; NeckPipelineOutput outputs[3];
    NeckPostureContext detector; NeckPostureParameters dp; NeckPostureOutput alarm;
    FILE *csv=fopen(path,"w"); unsigned n,i,pair,captures=0,triggers=0,changes=0,clears=0;
    ++groups; CHECK(csv!=NULL); neck_relative_default_parameters(&p);
    neck_pipeline_default_parameters(&pp); neck_posture_default_parameters(&dp);
    CHECK(neck_relative_init(&c,1,1,neck_simulator_mounts(),&p)==NECK_OK);
    CHECK(neck_posture_init(&detector,1,NECK_NODE_C4,&dp)==NECK_OK);
    CHECK(neck_simulator_mount(3)==NULL); neck_simulator_frame(NULL,0,0);
    for(i=0;i<3;++i) {
        CHECK(neck_pipeline_init(&pipelines[i],1,(uint8_t)i,neck_simulator_mount((uint8_t)i),.001,&pp)==NECK_OK);
        CHECK(neck_pipeline_begin_zero(&pipelines[i],1,NECK_REFERENCE_SIMULATION)==NECK_OK);
    }
    fprintf(csv,"sequence,time_us,pair,status,mask,flags,reference_id,raw_flexion,raw_lateral,raw_axial,corrected_flexion,corrected_lateral,corrected_axial,C4_alarm,C4_events\n");
    for(n=0;n<=4800;++n) {
        NeckAcquisitionFrame packet; NeckRelativeOutput o; int ready=1;
        neck_simulator_frame(&packet,n,(uint64_t)n*10000);
        CHECK(packet.acceleration.sequence==packet.orientation.sequence);
        for(i=0;i<3;++i) {
            Matrix sensor=quaternion_matrix(packet.orientation.nodes[i].sensor_to_world_wxyz);
            double norm=hypot(hypot(packet.acceleration.nodes[i].accel_m_s2[0],
                packet.acceleration.nodes[i].accel_m_s2[1]),packet.acceleration.nodes[i].accel_m_s2[2]);
            near(norm,9.80665);
            near(packet.acceleration.nodes[i].accel_m_s2[0],sensor.v[6]*9.80665);
            CHECK(neck_pipeline_process(&pipelines[i],&packet.acceleration,&outputs[i])==NECK_OK);
            if(outputs[i].zero.status!=NECK_ZERO_SUCCEEDED || outputs[i].filtered.reference_id!=1) ready=0;
        }
        CHECK(neck_posture_process(&detector,&outputs[1],&alarm)==NECK_OK);
        if(alarm.flexion.events&NECK_EVENT_TRIGGER) ++triggers;
        if(alarm.flexion.events&NECK_EVENT_LEVEL_CHANGE) ++changes;
        if(alarm.flexion.events&NECK_EVENT_CLEAR) ++clears;
        /* Orientation-only faults: S03/S04/S05 receive unchanged acceleration. */
        if(n==700) packet.orientation.nodes[0].quat_present=0;
        if(n==800) packet.orientation.nodes[1].sample_time_known=0;
        if(n==900) memset(packet.orientation.nodes[2].sensor_to_world_wxyz,0,4*sizeof(double));
        o=process(&c,&packet.orientation);
        if(!captures && ready) {
            CHECK(n==300); CHECK(neck_relative_capture_reference(&c,&o,1,0)==NECK_OK); ++captures;
            printf("relative reference committed at %.2f s (next-frame correction)\n",n*.01);
        }
        if(n<301) CHECK(o.reference_id==0);
        else CHECK(o.reference_id==1);
        if(n==0) { near(o.pairs[0].raw_euler_deg[0],7); near(o.pairs[1].raw_euler_deg[0],2);
            near(o.pairs[2].raw_euler_deg[0],5); near(o.pairs[3].raw_euler_deg[0],5); }
        if(n==301) for(pair=0;pair<4;++pair) near(o.pairs[pair].corrected_rotation_deg,0);
        for(pair=0;pair<4;++pair) {
            int bad=(n==700 && pair<2)||(n==800 && (pair==1||pair==2))||
                (n==900 && pair!=1);
            available(&o,pair,!bad);
            fprintf(csv,"%u,%llu,%u,%u,%u,%u,%u,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%u,%u\n",
                n,(unsigned long long)o.frame_time_us,pair,(unsigned)o.pairs[pair].status,
                (unsigned)o.pairs[pair].valid_mask,(unsigned)o.pairs[pair].flags,(unsigned)o.reference_id,
                o.pairs[pair].raw_euler_deg[0],o.pairs[pair].raw_euler_deg[1],o.pairs[pair].raw_euler_deg[2],
                o.pairs[pair].corrected_euler_deg[0],o.pairs[pair].corrected_euler_deg[1],o.pairs[pair].corrected_euler_deg[2],
                (unsigned)alarm.flexion.alarm_level,(unsigned)alarm.flexion.events);
        }
    }
    CHECK(fclose(csv)==0); CHECK(captures==1); CHECK(triggers==1 && changes==2 && clears==1);
    printf("C simulator integration: 4801 frames, 19204 pair rows, 3 orientation faults; C4 events %u/%u/%u\n",
        triggers,changes,clears);
}
int main(int argc, char **argv)
{
    CHECK(argc==2); test_arguments(); test_identity_and_axes(); test_matrix_grid_and_mounts();
    test_common_world_rotation(); test_sign_and_normalization(); test_invalid_quaternions();
    test_node_metadata(); test_timing_boundaries(); test_gimbal_lock(); test_reference_rotation();
    test_stream_provenance(); test_large_time_and_gaps(); test_c_simulator_and_pipeline(argv[1]);
    printf("matrix oracle maximum absolute element error: %.17g\n",max_matrix_error);
    printf("PASS: %u groups, %u C assertions; no Python runtime\n",groups,checks);
    return 0;
}
