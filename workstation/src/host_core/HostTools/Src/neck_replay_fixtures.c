#include "neck_replay_internal.h"
#include "neck_simulator.h"
#include <math.h>
#include <string.h>
static void declared_pose(uint64_t t,int motion,NeckReplayTruth *truth)
{
    double s=(double)t*1e-6,cycle=fmod(s,60),m=cycle>5?cycle-5:0;
    double p=cycle<5?10:(cycle<20?35:(cycle<40?50:10)),r=0,y=8*sin(m*.2),b=2+5*sin(m*.25);
    if(motion) { m=s>5?s-5:0; p=10+20*sin(m*.4); r=8*sin(m*.7); y=8*sin(m*.2); b=2+5*sin(m*.25); }
    memset(truth,0,sizeof(*truth)); truth->time_us=t; truth->source=NECK_SOURCE_SIMULATION;
    truth->euler_deg[0][0]=p+b; truth->euler_deg[0][1]=r+3*sin(m*.35); truth->euler_deg[0][2]=y+5*sin(m*.15);
    truth->euler_deg[1][0]=p; truth->euler_deg[1][1]=r; truth->euler_deg[1][2]=y;
    truth->euler_deg[2][0]=5; truth->euler_deg[2][2]=y;
}
static void motion_frame(NeckAcquisitionFrame *f,const NeckReplayTruth *t)
{
    unsigned i; const double h=.00872664625997164788;
    neck_simulator_frame(f,t->time_us/10000,t->time_us); /* envelope only */
    for(i=0;i<3;++i) {
        const double *m=neck_simulator_mount((uint8_t)i); double *q=f->orientation.nodes[i].sensor_to_world_wxyz;
        double cp=cos(t->euler_deg[i][0]*h),sp=sin(t->euler_deg[i][0]*h),cr=cos(t->euler_deg[i][1]*h),sr=sin(t->euler_deg[i][1]*h);
        double cy=cos(t->euler_deg[i][2]*h),sy=sin(t->euler_deg[i][2]*h),b[4],w,x,y,z;
        b[0]=cy*cp*cr+sy*sp*sr; b[1]=cy*cp*sr-sy*sp*cr; b[2]=cy*sp*cr+sy*cp*sr; b[3]=sy*cp*cr-cy*sp*sr;
        q[0]=b[0]*m[0]-b[1]*m[1]-b[2]*m[2]-b[3]*m[3];
        q[1]=b[0]*m[1]+b[1]*m[0]+b[2]*m[3]-b[3]*m[2];
        q[2]=b[0]*m[2]-b[1]*m[3]+b[2]*m[0]+b[3]*m[1];
        q[3]=b[0]*m[3]+b[1]*m[2]-b[2]*m[1]+b[3]*m[0];
        w=q[0]; x=q[1]; y=q[2]; z=q[3];
        f->acceleration.nodes[i].accel_m_s2[0]=9.80665*2*(x*z-w*y);
        f->acceleration.nodes[i].accel_m_s2[1]=9.80665*2*(y*z+w*x);
        f->acceleration.nodes[i].accel_m_s2[2]=9.80665*(1-2*(x*x+y*y));
    }
}
int neck_replay_generate(const char *scenario,const char *dir,NeckReplayError *e)
{
    FILE *in=NULL,*cf=NULL,*tf=NULL,*note=NULL; NeckReplayConfig config; NeckReplayEvent ev;
    NeckReplayTruth truth; int kind,status; unsigned n,i;
    if(e!=NULL) memset(e,0,sizeof(*e));
    if(scenario==NULL) return nr_error(e,NR_ARGUMENT,"Missing scenario.");
    kind=!strcmp(scenario,"baseline")?0:(!strcmp(scenario,"silence_faults")?1:(!strcmp(scenario,"timing_faults")?2:(!strcmp(scenario,"motion")?3:-1)));
    if(kind<0) return nr_error(e,NR_ARGUMENT,"Scenario: baseline | silence_faults | timing_faults | motion.");
#if NECK_SIM_PROFILE != NECK_SIM_SUSTAINED
    return nr_error(e,NR_CONFIG,"Fixtures require the default SUSTAINED simulator build.");
#endif
    status=nr_new_directory(dir,e); if(status!=NR_OK) return status;
    in=nr_new_file(dir,"inputs.csv",e); cf=nr_new_file(dir,"config.csv",e); tf=nr_new_file(dir,"truth.csv",e); note=nr_new_file(dir,"scenario.txt",e);
    if(!in || !cf || !tf || !note) { status=NR_IO; goto done; }
    neck_replay_default_config(&config); status=neck_replay_config_write(cf,&config);
    fprintf(in,"%s\n",NECK_REPLAY_INPUT_HEADER); fprintf(tf,"%s\n",NECK_REPLAY_TRUTH_HEADER);
    fprintf(note,"scenario=%s\nmode=DETERMINISTIC_C_SIMULATION\nduration_us=65000000\nperiod_us=10000\ncommands=ZERO_SIMULATION_AT_0;POLL_BEFORE_EACH_FRAME\ntruth=DECLARED_BODY_RZ_RY_RX_EULER_NOT_PHYSICAL_REFERENCE\n",scenario);
    if(kind==1) fputs("faults=32.00..32.49_NO_FRAME;36.00_C4_ACCEL_NAN;38.00_C0_QUAT_MISSING\n",note);
    if(kind==2) fputs("faults=17.00..17.49_C4_SAMPLE_TIME_FROZEN_16.99;24.00..24.04_C7_WORLD_2;28.00_FRAME_40MS_OLD;45.00_DUPLICATE_SEQUENCE;50.00_FRAME_1MS_FUTURE\n",note);
    if(kind==3) fputs("faults=8.00..8.49_C4_ACCEL_NORM_X1.3;20.00..20.49_C4_ACCEL_SAME_NORM_WRONG_DIRECTION;40.00_ILLEGAL_ACCEL_CAL4\nlimitation=SAME_NORM_DYNAMIC_ACCEL_CAN_PASS_SOFTWARE_GATE_AND_BIAS_GRAVITY_PROXY\n",note);
    memset(&ev,0,sizeof(ev)); ev.kind=NR_ZERO; status=neck_replay_event_write(in,&ev);
    for(n=0;status==NR_OK && n<=6500;++n) {
        ev.kind=NR_POLL; ev.now_us=(uint64_t)n*10000; status=neck_replay_event_write(in,&ev);
        if(status!=NR_OK) break;
        declared_pose(ev.now_us,kind==3,&truth); status=neck_replay_truth_write(tf,&truth); if(status!=NR_OK) break;
        if(kind==1 && n>=3200 && n<3250) continue;
        ev.kind=NR_FRAME; neck_simulator_frame(&ev.frame,n,ev.now_us);
        if(kind==3) motion_frame(&ev.frame,&truth);
        if(kind==1) {
            if(n==3600) ev.frame.acceleration.nodes[1].accel_m_s2[0]=NAN;
            if(n==3800) ev.frame.orientation.nodes[0].quat_present=0;
        } else if(kind==2) {
            if(n>=1700 && n<1750) ev.frame.acceleration.nodes[1].sample_time_us=ev.frame.orientation.nodes[1].sample_time_us=16990000;
            if(n>=2400 && n<2405) ev.frame.orientation.nodes[2].world_frame_id=2;
            if(n==2800) neck_simulator_frame(&ev.frame,n,27960000);
            if(n==4500) ev.frame.acceleration.sequence=ev.frame.orientation.sequence=n-1;
            if(n==5000) neck_simulator_frame(&ev.frame,n,50001000);
        } else if(kind==3) {
            if(n>=800 && n<850) for(i=0;i<3;++i) ev.frame.acceleration.nodes[1].accel_m_s2[i]*=1.3;
            if(n>=2000 && n<2050) {
                ev.frame.acceleration.nodes[1].accel_m_s2[0]=-.6*9.80665;
                ev.frame.acceleration.nodes[1].accel_m_s2[1]=0;
                ev.frame.acceleration.nodes[1].accel_m_s2[2]=.8*9.80665;
            }
            if(n==4000) ev.frame.acceleration.nodes[1].calibration_accel=4;
        }
        status=neck_replay_event_write(in,&ev);
    }
done:
    if(nr_close(in,e)!=NR_OK) status=NR_IO;
    if(nr_close(cf,e)!=NR_OK) status=NR_IO;
    if(nr_close(tf,e)!=NR_OK) status=NR_IO;
    if(nr_close(note,e)!=NR_OK) status=NR_IO;
    if(status!=NR_OK) {
        FILE *failed=nr_new_file(dir,"FAILED.txt",NULL);
        if(failed!=NULL) { fputs("PARTIAL DATASET: generation failed.\n",failed); (void)nr_close(failed,NULL); }
    }
    return status;
}
