#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200809L
#endif
#include "neck_replay_internal.h"
#include "neck_simulator.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#if defined(_WIN32)
#include <direct.h>
#include <io.h>
#else
#include <unistd.h>
#endif

int nr_error(NeckReplayError *e, int code, const char *message)
{ if(e!=NULL) (void)snprintf(e->message,sizeof(e->message),"%s",message); return code; }
void nr_double(FILE *f, double x)
{ if(isnan(x)) fputs("nan",f); else if(isinf(x)) fputs(x<0?"-inf":"inf",f); else fprintf(f,"%.17g",x); }
void nr_optional(FILE *f, double x, int valid)
{ if(valid && isfinite(x)) nr_double(f,x); }
int nr_close(FILE *f, NeckReplayError *e)
{ int failed; if(f==NULL) return NR_OK; failed=ferror(f); if(fclose(f)!=0) failed=1; return failed?nr_error(e,NR_IO,"File write/read error."):NR_OK; }
int nr_path(char out[1024], const char *dir, const char *name, NeckReplayError *e)
{
    int n;
    if(dir==NULL || name==NULL || !*dir) return nr_error(e,NR_ARGUMENT,"Empty directory.");
    n=snprintf(out,1024,"%s/%s",dir,name);
    return n<0 || n>=1024?nr_error(e,NR_LIMIT,"Path is too long."):NR_OK;
}
int nr_new_directory(const char *path, NeckReplayError *e)
{
    int r;
    if(path==NULL || !*path) return nr_error(e,NR_ARGUMENT,"Empty output directory.");
#if defined(_WIN32)
    r=_mkdir(path);
#else
    r=mkdir(path,0700);
#endif
    if(r==0) return NR_OK;
    return nr_error(e,errno==EEXIST?NR_EXISTS:NR_IO,"Cannot create NEW directory (existing runs are never overwritten).");
}
FILE *nr_new_file(const char *directory, const char *name, NeckReplayError *e)
{
    char path[1024]; FILE *f; int fd;
    if(nr_path(path,directory,name,e)!=NR_OK) return NULL;
#if defined(_WIN32)
    fd=_open(path,_O_WRONLY|_O_CREAT|_O_EXCL|_O_BINARY,_S_IREAD|_S_IWRITE);
    f=fd<0?NULL:_fdopen(fd,"wb");
    if(fd>=0 && f==NULL) _close(fd);
#else
    fd=open(path,O_WRONLY|O_CREAT|O_EXCL,0600);
    f=fd<0?NULL:fdopen(fd,"wb");
    if(fd>=0 && f==NULL) close(fd);
#endif
    if(f==NULL) nr_error(e,NR_IO,"Cannot create exclusive output file.");
    return f;
}
int nr_read_line(FILE *f, char out[NR_LINE_MAX], unsigned long *number, uint64_t *hash, NeckReplayError *e)
{
    size_t n=0; int ch,any=0,bad=0;
    while((ch=fgetc(f))!=EOF) {
        any=1; *hash^=(unsigned char)ch; *hash*=UINT64_C(1099511628211);
        if(ch=='\n') break;
        if(ch==0) bad=1;
        if(n+1<NR_LINE_MAX) out[n++]=(char)ch; else bad=1;
    }
    if(ferror(f)) { nr_error(e,NR_IO,"Input read failed."); return -1; }
    if(!any) return 0;
    ++*number; if(e!=NULL) e->line=*number;
    if(bad) { nr_error(e,NR_FORMAT,"NUL byte or line exceeds 4095 bytes."); return -1; }
    if(n && out[n-1]=='\r') --n;
    out[n]='\0'; return 1;
}
int nr_header(FILE *f, const char *expected, unsigned long *line, uint64_t *hash, NeckReplayError *e)
{
    char text[NR_LINE_MAX]; const unsigned char *p=(const unsigned char *)text;
    int r=nr_read_line(f,text,line,hash,e);
    if(r!=1) return nr_error(e,NR_FORMAT,"Missing schema header.");
    if(strlen(text)>=3 && p[0]==239 && p[1]==187 && p[2]==191) p+=3;
    return strcmp((const char *)p,expected)?nr_error(e,NR_FORMAT,"Unsupported schema header."):NR_OK;
}
static int split(char *line, char *parts[96], unsigned *count, NeckReplayError *e)
{
    char *p; unsigned n=1; parts[0]=line;
    for(p=line;*p;++p) {
        if(*p=='"' || *p=='\r' || *p=='\n') return nr_error(e,NR_FORMAT,"Quoted/multiline fields are not part of typed CSV v1.");
        if(*p==',') { *p='\0'; if(n==96) return nr_error(e,NR_FORMAT,"Too many columns."); parts[n++]=p+1; }
    }
    *count=n; return NR_OK;
}
static int uint_text(const char *s, uint64_t limit, uint64_t *out, NeckReplayError *e)
{
    const char *p=s; unsigned long long value; char *end;
    if(!*p) return nr_error(e,NR_FORMAT,"Missing unsigned decimal integer.");
    for(;*p;++p) if(*p<'0' || *p>'9') return nr_error(e,NR_FORMAT,"Unsigned integers require decimal digits only.");
    errno=0; value=strtoull(s,&end,10);
    if(errno==ERANGE || *end || value>limit) return nr_error(e,NR_FORMAT,"Integer exceeds its field width.");
    *out=(uint64_t)value; return NR_OK;
}
static int double_text(const char *s, int fault_allowed, double *out, NeckReplayError *e)
{
    const char *p=s; char *end; unsigned digits=0;
    if(fault_allowed && (!strcmp(s,"nan") || !strcmp(s,"inf") || !strcmp(s,"-inf"))) {
        *out=!strcmp(s,"nan")?NAN:(!strcmp(s,"inf")?INFINITY:-INFINITY); return NR_OK;
    }
    if(*p=='-') ++p;
    while(*p>='0' && *p<='9') { ++p; ++digits; }
    if(*p=='.') { ++p; while(*p>='0' && *p<='9') { ++p; ++digits; } }
    if(!digits) return nr_error(e,NR_FORMAT,"Missing finite decimal number.");
    if(*p=='e' || *p=='E') { unsigned exponent=0; ++p; if(*p=='+' || *p=='-') ++p;
        while(*p>='0' && *p<='9') { ++p; ++exponent; }
        if(!exponent) return nr_error(e,NR_FORMAT,"Missing decimal exponent."); }
    if(*p) return nr_error(e,NR_FORMAT,"Invalid decimal number suffix.");
    errno=0; *out=strtod(s,&end);
    /* Representable subnormals can set ERANGE. Preserve them; reject overflow
     * and nonzero literals that underflow to zero. The S07 gate decides whether
     * an extremely small sensor vector is usable. */
    if(*end || !isfinite(*out) || (errno==ERANGE && *out==0)) return nr_error(e,NR_FORMAT,"Decimal number out of range (use explicit nan/inf for sensor faults).");
    return NR_OK;
}
#define U64_READ(member) do { if(uint_text(p[n++],UINT64_MAX,&v,e)!=NR_OK) return NR_FORMAT; (member)=v; } while(0)
#define U32_READ(member) do { if(uint_text(p[n++],UINT32_MAX,&v,e)!=NR_OK) return NR_FORMAT; (member)=(uint32_t)v; } while(0)
#define U8_READ(member) do { if(uint_text(p[n++],UINT8_MAX,&v,e)!=NR_OK) return NR_FORMAT; (member)=(uint8_t)v; } while(0)
int neck_replay_event_parse(const char *text, NeckReplayEvent *event, NeckReplayError *e)
{
    char line[NR_LINE_MAX],*p[96]; unsigned count,n=1,i,j; uint64_t v;
    if(text==NULL || event==NULL) return nr_error(e,NR_ARGUMENT,"Null event argument.");
    memset(event,0,sizeof(*event));
    if(strlen(text)>=sizeof(line)) return nr_error(e,NR_FORMAT,"Event line too long.");
    strcpy(line,text); if(split(line,p,&count,e)!=NR_OK) return NR_FORMAT;
    if(!strcmp(p[0],"FRAME")) event->kind=NR_FRAME;
    else if(!strcmp(p[0],"POLL")) event->kind=NR_POLL;
    else if(!strcmp(p[0],"ZERO")) event->kind=NR_ZERO;
    else if(!strcmp(p[0],"CANCEL")) event->kind=NR_CANCEL;
    else if(!strcmp(p[0],"RESET")) event->kind=NR_RESET;
    else return nr_error(e,NR_FORMAT,"Unknown event kind.");
    if(count!=(event->kind==NR_FRAME?69u:(event->kind==NR_ZERO?3u:2u)))
        return nr_error(e,NR_FORMAT,"Wrong number of event columns.");
    U64_READ(event->now_us);
    if(event->kind==NR_ZERO) { U8_READ(event->origin); if(event->origin>1) return nr_error(e,NR_FORMAT,"ZERO origin must be 0 or 1."); }
    if(event->kind!=NR_FRAME) return NR_OK;
    U32_READ(event->frame.acceleration.config_id); U32_READ(event->frame.acceleration.session_id);
    U64_READ(event->frame.acceleration.sequence); U64_READ(event->frame.acceleration.frame_time_us); U8_READ(event->frame.acceleration.source);
    U32_READ(event->frame.orientation.config_id); U32_READ(event->frame.orientation.session_id);
    U64_READ(event->frame.orientation.sequence); U64_READ(event->frame.orientation.frame_time_us); U8_READ(event->frame.orientation.source);
    for(i=0;i<3;++i) {
        NeckNodeSample *a=&event->frame.acceleration.nodes[i]; NeckOrientationSample *q=&event->frame.orientation.nodes[i];
        for(j=0;j<3;++j) if(double_text(p[n++],1,&a->accel_m_s2[j],e)!=NR_OK) return NR_FORMAT;
        U64_READ(a->sample_time_us); U8_READ(a->signal_valid_mask); U8_READ(a->accel_present);
        U8_READ(a->sample_time_known); U8_READ(a->calibration_accel); U8_READ(a->status);
        for(j=0;j<4;++j) if(double_text(p[n++],1,&q->sensor_to_world_wxyz[j],e)!=NR_OK) return NR_FORMAT;
        U64_READ(q->sample_time_us); U32_READ(q->world_frame_id); U8_READ(q->quat_present);
        U8_READ(q->sample_time_known); U8_READ(q->calibration_system); U8_READ(q->status);
    }
    return NR_OK;
}
int neck_replay_event_write(FILE *f, const NeckReplayEvent *ev)
{
    static const char *const names[6]={"","FRAME","POLL","ZERO","CANCEL","RESET"}; unsigned i,j;
    if(f==NULL || ev==NULL || ev->kind<1 || ev->kind>5) return NR_ARGUMENT;
    fprintf(f,"%s,%llu",names[ev->kind],(unsigned long long)ev->now_us);
    if(ev->kind==NR_ZERO) fprintf(f,",%u",(unsigned)ev->origin);
    if(ev->kind==NR_FRAME) {
        const NeckFrame *a=&ev->frame.acceleration; const NeckOrientationFrame *q=&ev->frame.orientation;
        fprintf(f,",%u,%u,%llu,%llu,%u,%u,%u,%llu,%llu,%u",(unsigned)a->config_id,(unsigned)a->session_id,
            (unsigned long long)a->sequence,(unsigned long long)a->frame_time_us,(unsigned)a->source,
            (unsigned)q->config_id,(unsigned)q->session_id,(unsigned long long)q->sequence,(unsigned long long)q->frame_time_us,(unsigned)q->source);
        for(i=0;i<3;++i) {
            const NeckNodeSample *an=&a->nodes[i]; const NeckOrientationSample *qn=&q->nodes[i];
            for(j=0;j<3;++j) { fputc(',',f); nr_double(f,an->accel_m_s2[j]); }
            fprintf(f,",%llu,%u,%u,%u,%u,%u",(unsigned long long)an->sample_time_us,(unsigned)an->signal_valid_mask,
                (unsigned)an->accel_present,(unsigned)an->sample_time_known,(unsigned)an->calibration_accel,(unsigned)an->status);
            for(j=0;j<4;++j) { fputc(',',f); nr_double(f,qn->sensor_to_world_wxyz[j]); }
            fprintf(f,",%llu,%u,%u,%u,%u,%u",(unsigned long long)qn->sample_time_us,(unsigned)qn->world_frame_id,
                (unsigned)qn->quat_present,(unsigned)qn->sample_time_known,(unsigned)qn->calibration_system,(unsigned)qn->status);
        }
    }
    fputc('\n',f); return ferror(f)?NR_IO:NR_OK;
}

enum { FIELD_DOUBLE=0, FIELD_U64=1, FIELD_U32=2, FIELD_U8=3 };
typedef struct { const char *key; size_t offset; unsigned type; } Field;
#define F(key,type,member) {key,offsetof(NeckReplayConfig,member),type}
#define Q(key,type,member) F("q_" key,type,parameters.quality.member)
#define P(key,type,member) F("lp_" key,type,parameters.pipeline.member)
#define A(key,type,member) F("alarm_" key,type,parameters.posture.member)
#define R(key,type,member) F("rel_" key,type,parameters.relative.member)
static const Field fields[]={
    F("config_id",FIELD_U32,config_id),F("world_frame_id",FIELD_U32,world_frame_id),F("start_wall_us",FIELD_U64,start_wall_us),
    F("mount_C0_w",0,mounts[0][0]),F("mount_C0_x",0,mounts[0][1]),F("mount_C0_y",0,mounts[0][2]),F("mount_C0_z",0,mounts[0][3]),
    F("mount_C4_w",0,mounts[1][0]),F("mount_C4_x",0,mounts[1][1]),F("mount_C4_y",0,mounts[1][2]),F("mount_C4_z",0,mounts[1][3]),
    F("mount_C7_w",0,mounts[2][0]),F("mount_C7_x",0,mounts[2][1]),F("mount_C7_y",0,mounts[2][2]),F("mount_C7_z",0,mounts[2][3]),
    Q("gravity_m_s2",0,gravity_m_s2),Q("norm_reentry_error_m_s2",0,norm_reentry_error_m_s2),Q("norm_exit_error_m_s2",0,norm_exit_error_m_s2),
    Q("quaternion_norm_tolerance",0,quaternion_norm_tolerance),Q("max_frame_age_us",1,max_frame_age_us),Q("max_sample_age_us",1,max_sample_age_us),
    Q("max_gap_us",1,max_gap_us),Q("no_frame_timeout_us",1,no_frame_timeout_us),Q("recovery_us",1,recovery_us),
    Q("minimum_accel_calibration",3,minimum_accel_calibration),Q("minimum_system_calibration",3,minimum_system_calibration),Q("allow_simulation_uncalibrated",3,allow_simulation_uncalibrated),
    P("cutoff_hz",0,cutoff_hz),P("nominal_period_us",1,nominal_period_us),P("max_gap_us",1,max_gap_us),P("warmup_us",1,warmup_us),
    P("zero_duration_us",1,zero_duration_us),P("zero_timeout_us",1,zero_timeout_us),P("zero_min_samples",2,zero_min_samples),
    P("gravity_m_s2",0,gravity_m_s2),P("gravity_tolerance_m_s2",0,gravity_tolerance_m_s2),P("norm_span_m_s2",0,norm_span_m_s2),P("angle_span_deg",0,angle_span_deg),
    A("flexion_warning_deg",0,flexion_warning_deg),A("flexion_danger_deg",0,flexion_danger_deg),A("lateral_warning_deg",0,lateral_warning_deg),A("lateral_danger_deg",0,lateral_danger_deg),
    A("continuous_duration_us",1,continuous_duration_us),A("max_gap_us",1,max_gap_us),R("quaternion_norm_tolerance",0,quaternion_norm_tolerance),
    R("gimbal_cos_min",0,gimbal_cos_min),R("stale_after_us",1,stale_after_us),R("max_node_skew_us",1,max_node_skew_us)
};
void neck_replay_default_config(NeckReplayConfig *c)
{
    if(c==NULL) return;
    memset(c,0,sizeof(*c)); c->config_id=c->world_frame_id=1;
    neck_processing_default_parameters(&c->parameters); memcpy(c->mounts,neck_simulator_mounts(),sizeof(c->mounts));
}
int neck_replay_config_write(FILE *f, const NeckReplayConfig *c)
{
    size_t i; if(f==NULL || c==NULL) return NR_ARGUMENT;
    fprintf(f,"%s\n",NECK_REPLAY_CONFIG_HEADER);
    for(i=0;i<sizeof(fields)/sizeof(fields[0]);++i) {
        const unsigned char *p=(const unsigned char *)c+fields[i].offset;
        fprintf(f,"%s,",fields[i].key);
        if(fields[i].type==0) nr_double(f,*(const double *)p);
        else if(fields[i].type==1) fprintf(f,"%llu",(unsigned long long)*(const uint64_t *)p);
        else if(fields[i].type==2) fprintf(f,"%u",(unsigned)*(const uint32_t *)p);
        else fprintf(f,"%u",(unsigned)*p);
        fputc('\n',f);
    }
    return ferror(f)?NR_IO:NR_OK;
}
int neck_replay_config_read(const char *path, NeckReplayConfig *c, uint64_t *digest, NeckReplayError *e)
{
    FILE *f; unsigned long number=0; char line[NR_LINE_MAX],*parts[96]; unsigned count; size_t i;
    uint8_t seen[sizeof(fields)/sizeof(fields[0])]; uint64_t hash=NR_FNV_OFFSET,value; int r=0,status;
    NeckProcessingContext check;
    if(path==NULL || c==NULL || digest==NULL) return nr_error(e,NR_ARGUMENT,"Null config argument.");
    memset(c,0,sizeof(*c)); memset(seen,0,sizeof(seen)); f=fopen(path,"rb");
    if(f==NULL) return nr_error(e,NR_IO,"Cannot open config.");
    status=nr_header(f,NECK_REPLAY_CONFIG_HEADER,&number,&hash,e);
    while(status==NR_OK && (r=nr_read_line(f,line,&number,&hash,e))==1) {
        unsigned char *target;
        if(split(line,parts,&count,e)!=NR_OK || count!=2) { status=nr_error(e,NR_FORMAT,"Config needs key,value rows."); break; }
        for(i=0;i<sizeof(seen) && strcmp(fields[i].key,parts[0]);++i) { }
        if(i==sizeof(seen) || seen[i]) { status=nr_error(e,NR_FORMAT,"Unknown or duplicate config key."); break; }
        seen[i]=1; target=(unsigned char *)c+fields[i].offset;
        if(fields[i].type==0) status=double_text(parts[1],0,(double *)target,e);
        else {
            status=uint_text(parts[1],fields[i].type==1?UINT64_MAX:(fields[i].type==2?UINT32_MAX:UINT8_MAX),&value,e);
            if(status==NR_OK) {
                if(fields[i].type==1) *(uint64_t *)target=value;
                else if(fields[i].type==2) *(uint32_t *)target=(uint32_t)value;
                else *target=(uint8_t)value;
            }
        }
    }
    if(status==NR_OK && ferror(f)) status=nr_error(e,NR_IO,"Config read failed.");
    if(status==NR_OK && r<0) status=NR_FORMAT;
    if(fclose(f)!=0 && status==NR_OK) status=nr_error(e,NR_IO,"Config close failed.");
    for(i=0;status==NR_OK && i<sizeof(seen);++i) if(!seen[i]) status=nr_error(e,NR_FORMAT,"Missing config key: no implicit defaults on replay.");
    if(status==NR_OK && neck_processing_init(&check,c->config_id,c->world_frame_id,
        (const double (*)[4])c->mounts,c->start_wall_us,&c->parameters)!=NECK_OK)
        status=nr_error(e,NR_CONFIG,"Configuration rejected by shared processing chain.");
    *digest=hash; return status;
}
int neck_replay_truth_parse(const char *text, NeckReplayTruth *t, NeckReplayError *e)
{
    char line[NR_LINE_MAX],*p[96]; unsigned count,n=0,i,j; uint64_t v;
    if(text==NULL || t==NULL) return nr_error(e,NR_ARGUMENT,"Null truth argument.");
    memset(t,0,sizeof(*t)); if(strlen(text)>=sizeof(line)) return nr_error(e,NR_FORMAT,"Truth line too long.");
    strcpy(line,text); if(split(line,p,&count,e)!=NR_OK || count!=11) return nr_error(e,NR_FORMAT,"Truth needs 11 columns.");
    U64_READ(t->time_us); U8_READ(t->source);
    if(t->source!=NECK_SOURCE_SIMULATION) return nr_error(e,NR_FORMAT,"Truth v1 is SIMULATION CONSTRUCTION ONLY.");
    for(i=0;i<3;++i) for(j=0;j<3;++j) {
        if(double_text(p[n++],0,&t->euler_deg[i][j],e)!=NR_OK) return NR_FORMAT;
        if(fabs(t->euler_deg[i][j])>180) return nr_error(e,NR_FORMAT,"Truth Euler outside [-180,180].");
    }
    return NR_OK;
}
int neck_replay_truth_write(FILE *f, const NeckReplayTruth *t)
{
    unsigned i,j; if(f==NULL || t==NULL) return NR_ARGUMENT;
    fprintf(f,"%llu,%u",(unsigned long long)t->time_us,(unsigned)t->source);
    for(i=0;i<3;++i) for(j=0;j<3;++j) { fputc(',',f); nr_double(f,t->euler_deg[i][j]); }
    fputc('\n',f); return ferror(f)?NR_IO:NR_OK;
}
void neck_replay_ideal_proxy(double pitch, double roll, double out[2], uint8_t *mask)
{
    const double rad=.01745329251994329577,deg=57.2957795130823208768;
    double forward=sin(pitch*rad),side=cos(pitch*rad)*sin(roll*rad),up=cos(pitch*rad)*cos(roll*rad);
    if(out==NULL || mask==NULL) return;
    out[0]=out[1]=0; *mask=0;
    if(!isfinite(forward) || !isfinite(side) || !isfinite(up) || up<=0) return;
    out[1]=asin(fmax(-1.0,fmin(1.0,side)))*deg; *mask=2;
    if(hypot(forward,side)<=NECK_DIRECTION_EPS) { out[0]=0; *mask|=1; }
    else if(fabs(forward)>NECK_DIRECTION_EPS) { out[0]=copysign(acos(fmax(-1.0,fmin(1.0,up)))*deg,forward); *mask|=1; }
}
int nr_truths_read(const char *path, NRTruths *t, NeckReplayError *e)
{
    FILE *f; char line[NR_LINE_MAX]; unsigned long number=0; int status,r=0;
    memset(t,0,sizeof(*t)); t->digest=NR_FNV_OFFSET; if(path==NULL || !*path) return NR_OK;
    f=fopen(path,"rb"); if(f==NULL) return nr_error(e,NR_IO,"Cannot open optional truth.");
    status=nr_header(f,NECK_REPLAY_TRUTH_HEADER,&number,&t->digest,e);
    while(status==NR_OK && (r=nr_read_line(f,line,&number,&t->digest,e))==1) {
        NeckReplayTruth row;
        status=neck_replay_truth_parse(line,&row,e); if(status!=NR_OK) break;
        if(t->count && row.time_us<=t->items[t->count-1].time_us) { status=nr_error(e,NR_FORMAT,"Truth times must strictly increase."); break; }
        if(t->count==NR_POINT_MAX) { status=nr_error(e,NR_LIMIT,"Truth point limit reached."); break; }
        if(t->count==t->capacity) {
            size_t cap=t->capacity?t->capacity*2:1024; NeckReplayTruth *items;
            if(cap>NR_POINT_MAX) cap=NR_POINT_MAX;
            items=(NeckReplayTruth *)realloc(t->items,cap*sizeof(*items));
            if(items==NULL) { status=nr_error(e,NR_IO,"Truth allocation failed (host only)."); break; }
            t->items=items; t->capacity=cap;
        }
        t->items[t->count++]=row;
    }
    if(status==NR_OK && r<0) status=NR_FORMAT;
    if(fclose(f)!=0 && status==NR_OK) status=nr_error(e,NR_IO,"Truth close failed.");
    return status;
}
const NeckReplayTruth *nr_truth_at(const NRTruths *t, uint64_t time, uint8_t source)
{
    size_t lo=0,hi=t->count;
    if(source!=NECK_SOURCE_SIMULATION) return NULL;
    while(lo<hi) { size_t mid=lo+(hi-lo)/2; if(t->items[mid].time_us<time) lo=mid+1; else hi=mid; }
    return lo<t->count && t->items[lo].time_us==time?&t->items[lo]:NULL;
}
