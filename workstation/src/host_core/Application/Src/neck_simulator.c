#include "neck_simulator.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static const double mounts[NECK_NODE_COUNT][4]={
    {0.7071067811865475244,0.7071067811865475244,0,0}, /* C0: sensor -> body Rx90 */
    {1,0,0,0},
    {0.7071067811865475244,0,0,0.7071067811865475244} /* C7: Rz90 */
};
const double *neck_simulator_mount(uint8_t node_id)
{ return node_id<NECK_NODE_COUNT?mounts[node_id]:NULL; }
const double (*neck_simulator_mounts(void))[4] { return mounts; }
static void quaternion(double pitch, double roll, double yaw, double q[4])
{
    const double half_deg=0.00872664625997164788;
    double cp=cos(pitch*half_deg),sp=sin(pitch*half_deg);
    double cr=cos(roll*half_deg),sr=sin(roll*half_deg);
    double cy=cos(yaw*half_deg),sy=sin(yaw*half_deg);
    q[0]=cy*cp*cr+sy*sp*sr; q[1]=cy*cp*sr-sy*sp*cr;
    q[2]=cy*sp*cr+sy*cp*sr; q[3]=sy*cp*cr-cy*sp*sr;
}
static void multiply(const double a[4], const double b[4], double q[4])
{
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    q[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
void neck_simulator_frame(NeckAcquisitionFrame *packet, uint64_t sequence, uint64_t time_us)
{
    double seconds=(double)time_us*1.0e-6;
#if NECK_SIM_PROFILE == NECK_SIM_SUSTAINED
    double cycle=fmod(seconds,60.0), moving=cycle>5.0?cycle-5.0:0.0;
    double pitch=cycle<5.0?10.0:(cycle<20.0?35.0:(cycle<40.0?50.0:10.0));
    double side=0.0;
#else
    double moving=seconds>5.0?seconds-5.0:0.0;
    double pitch=10.0+15.0*sin(moving*.4), side=5.0*sin(moving*.7);
#endif
    double yaw=8.0*sin(moving*.2), bend=2.0+5.0*sin(moving*.25);
    double pitches[3]={pitch+bend,pitch,5.0};
    double rolls[3]={side+3.0*sin(moving*.35),side,0.0};
    double yaws[3]={yaw+5.0*sin(moving*.15),yaw,yaw};
    unsigned i;
    if(packet==NULL) return;
    memset(packet,0,sizeof(*packet));
    packet->acceleration.config_id=packet->orientation.config_id=1;
    packet->acceleration.session_id=packet->orientation.session_id=1;
    packet->acceleration.sequence=packet->orientation.sequence=sequence;
    packet->acceleration.frame_time_us=packet->orientation.frame_time_us=time_us;
    packet->acceleration.source=packet->orientation.source=NECK_SOURCE_SIMULATION;
    for(i=0;i<NECK_NODE_COUNT;++i) {
        NeckNodeSample *a=&packet->acceleration.nodes[i];
        NeckOrientationSample *o=&packet->orientation.nodes[i];
        double body[4],w,x,y,z;
        quaternion(pitches[i],rolls[i],yaws[i],body);
        multiply(body,mounts[i],o->sensor_to_world_wxyz);
        w=o->sensor_to_world_wxyz[0]; x=o->sensor_to_world_wxyz[1];
        y=o->sensor_to_world_wxyz[2]; z=o->sensor_to_world_wxyz[3];
        /* R_world_sensor^T * [0,0,g]: specific force in sensor coordinates. */
        a->accel_m_s2[0]=9.80665*2.0*(x*z-w*y);
        a->accel_m_s2[1]=9.80665*2.0*(y*z+w*x);
        a->accel_m_s2[2]=9.80665*(1.0-2.0*(x*x+y*y));
        a->sample_time_us=o->sample_time_us=time_us;
        a->signal_valid_mask=NECK_SIGNAL_ACCEL|NECK_SIGNAL_QUATERNION;
        a->accel_present=a->sample_time_known=o->quat_present=o->sample_time_known=1;
        a->calibration_accel=o->calibration_system=NECK_CALIBRATION_UNKNOWN;
        a->status=o->status=NECK_NODE_PARTIAL;
        o->world_frame_id=1;
    }
}
