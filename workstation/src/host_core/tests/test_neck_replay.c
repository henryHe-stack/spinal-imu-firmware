/* S08 host C tests. Algorithm math/gates are regression-tested in S03--S07;
 * this suite tests lossless event replay, exports, failures and presentation. */
#include "neck_replay_internal.h"
#include "neck_simulator.h"
#include <locale.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
static unsigned groups,checks;
static char root[1024];
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while(0)
typedef struct { FILE *f; char header[NR_LINE_MAX],line[NR_LINE_MAX]; char *keys[96],*cells[96]; unsigned columns; } Csv;
static unsigned fields(char *s,char **out)
{
    unsigned n=1; char *p; out[0]=s;
    for(p=s;*p;++p) if(*p==',') { *p=0; if(n>=96) CHECK(0); out[n++]=p+1; }
    return n;
}
static int line_read(FILE *f,char *s)
{
    size_t n; if(fgets(s,NR_LINE_MAX,f)==NULL) return 0;
    n=strlen(s); while(n && (s[n-1]=='\n' || s[n-1]=='\r')) s[--n]=0; return 1;
}
static void path(char out[1024],const char *name)
{ CHECK(snprintf(out,1024,"%s/%s",root,name)>0); }
static FILE *new_test_file(const char *name)
{ char p[1024]; FILE *f; path(p,name); f=fopen(p,"wb"); CHECK(f!=NULL); return f; }
static int exists(const char *name)
{ char p[1024]; FILE *f; path(p,name); f=fopen(p,"rb"); if(f==NULL) return 0; CHECK(fclose(f)==0); return 1; }
static Csv csv_open(const char *name)
{
    Csv c; char p[1024]; memset(&c,0,sizeof(c)); path(p,name); c.f=fopen(p,"rb"); CHECK(c.f!=NULL);
    CHECK(line_read(c.f,c.header)); /* pointer fields must be initialized AFTER return, not into a copied stack struct */
    return c;
}
static void csv_keys(Csv *c) { c->columns=fields(c->header,c->keys); }
static int csv_next(Csv *c)
{ if(!line_read(c->f,c->line)) return 0; CHECK(fields(c->line,c->cells)==c->columns); return 1; }
static unsigned column(const Csv *c,const char *name)
{ unsigned i; for(i=0;i<c->columns;++i) if(!strcmp(c->keys[i],name)) return i; CHECK(0); return 0; }
static const char *get(const Csv *c,const char *name) { return c->cells[column(c,name)]; }
static uint64_t number(const Csv *c,const char *name) { return (uint64_t)strtoull(get(c,name),NULL,10); }
static char *all(const char *name)
{
    char p[1024],*data; long size; FILE *f;
    path(p,name); f=fopen(p,"rb"); CHECK(f!=NULL); CHECK(fseek(f,0,SEEK_END)==0); size=ftell(f); CHECK(size>=0);
    CHECK(fseek(f,0,SEEK_SET)==0); data=(char *)malloc((size_t)size+1); CHECK(data!=NULL);
    CHECK(fread(data,1,(size_t)size,f)==(size_t)size); data[size]=0; CHECK(fclose(f)==0); return data;
}
static void same_file(const char *a,const char *b)
{
    char pa[1024],pb[1024]; FILE *fa,*fb; unsigned char ba[4096],bb[4096]; size_t na,nb;
    path(pa,a); path(pb,b); fa=fopen(pa,"rb"); fb=fopen(pb,"rb"); CHECK(fa!=NULL && fb!=NULL);
    do { na=fread(ba,1,sizeof(ba),fa); nb=fread(bb,1,sizeof(bb),fb); CHECK(na==nb); CHECK(memcmp(ba,bb,na)==0); } while(na);
    CHECK(!ferror(fa) && !ferror(fb)); CHECK(fclose(fa)==0); CHECK(fclose(fb)==0);
}
static NeckReplayStats run(const char *dataset,const char *result,int truth)
{
    char in[1024],cf[1024],tf[1024],out[1024],s[256]; NeckReplayStats stats; NeckReplayError e={0,{0}};
    snprintf(s,sizeof(s),"%s/inputs.csv",dataset); path(in,s);
    snprintf(s,sizeof(s),"%s/config.csv",dataset); path(cf,s);
    snprintf(s,sizeof(s),"%s/truth.csv",dataset); path(tf,s); path(out,result);
    if(neck_replay_run(in,cf,truth?tf:NULL,out,&stats,&e)!=NR_OK) { fprintf(stderr,"%s line %lu: %s\n",result,e.line,e.message); CHECK(0); }
    return stats;
}
static void replace_field(const char *original,unsigned index,const char *replacement,char out[NR_LINE_MAX])
{
    char copy[NR_LINE_MAX],*p[96]; unsigned n,i; size_t len=0;
    strcpy(copy,original); n=fields(copy,p); CHECK(index<n); out[0]=0;
    for(i=0;i<n;++i) { const char *s=i==index?replacement:p[i]; size_t k=strlen(s); CHECK(len+k+2<NR_LINE_MAX);
        if(i) { out[len++]=','; }
        memcpy(out+len,s,k); len+=k; out[len]=0; }
}
static void codecs(void)
{
    NeckReplayEvent a,b; NeckReplayError e={0,{0}}; FILE *f; char text[NR_LINE_MAX],second[NR_LINE_MAX]; unsigned i;
    ++groups; memset(&a,0,sizeof(a)); a.kind=NR_FRAME; a.now_us=UINT64_MAX;
    neck_simulator_frame(&a.frame,UINT64_MAX,0);
    a.frame.acceleration.config_id=UINT32_MAX;
    a.frame.orientation.config_id=UINT32_MAX; a.frame.acceleration.session_id=a.frame.orientation.session_id=UINT32_MAX;
    a.frame.acceleration.frame_time_us=a.frame.orientation.frame_time_us=UINT64_MAX;
    for(i=0;i<3;++i) { a.frame.acceleration.nodes[i].sample_time_us=UINT64_MAX; a.frame.orientation.nodes[i].sample_time_us=UINT64_MAX; a.frame.orientation.nodes[i].world_frame_id=UINT32_MAX; }
    a.frame.acceleration.nodes[0].accel_m_s2[0]=-0.0; a.frame.acceleration.nodes[1].accel_m_s2[1]=NAN;
    a.frame.acceleration.nodes[2].accel_m_s2[2]=INFINITY; a.frame.orientation.nodes[2].sensor_to_world_wxyz[3]=-INFINITY;
    a.frame.acceleration.nodes[0].accel_m_s2[1]=DBL_MIN*.5; a.frame.acceleration.nodes[0].accel_m_s2[2]=DBL_MAX;
    f=tmpfile(); CHECK(f!=NULL); CHECK(neck_replay_event_write(f,&a)==NR_OK); rewind(f); CHECK(line_read(f,text)); CHECK(fclose(f)==0);
    CHECK(neck_replay_event_parse(text,&b,&e)==NR_OK); CHECK(b.now_us==UINT64_MAX); CHECK(b.frame.acceleration.sequence==UINT64_MAX);
    CHECK(b.frame.acceleration.config_id==UINT32_MAX); CHECK(b.frame.orientation.nodes[2].world_frame_id==UINT32_MAX);
    CHECK(signbit(b.frame.acceleration.nodes[0].accel_m_s2[0])); CHECK(isnan(b.frame.acceleration.nodes[1].accel_m_s2[1]));
    CHECK(b.frame.acceleration.nodes[0].accel_m_s2[1]==DBL_MIN*.5); CHECK(b.frame.acceleration.nodes[0].accel_m_s2[2]==DBL_MAX);
    CHECK(isinf(b.frame.acceleration.nodes[2].accel_m_s2[2])); CHECK(b.frame.orientation.nodes[2].sensor_to_world_wxyz[3]<0);
    f=tmpfile(); CHECK(f!=NULL); CHECK(neck_replay_event_write(f,&b)==NR_OK); rewind(f); CHECK(line_read(f,second)); CHECK(!strcmp(text,second)); CHECK(fclose(f)==0);
    CHECK(neck_replay_event_parse("POLL,9223372036854775809",&b,&e)==NR_OK); CHECK(b.now_us==UINT64_C(9223372036854775809));
    CHECK(neck_replay_event_parse("ZERO,0,1",&b,&e)==NR_OK && b.origin==1); CHECK(neck_replay_event_parse("CANCEL,0",&b,&e)==NR_OK && b.kind==NR_CANCEL);
    CHECK(neck_replay_event_parse("RESET,0",&b,&e)==NR_OK && b.kind==NR_RESET);
    ++groups;
    { static const char *const bad[]={"","X,0","POLL","POLL,0,0","POLL,-1","POLL,+1","POLL,1e6","POLL,18446744073709551616","POLL, 0","POLL,0 ","POLL,\"0\"","POLL,0\n","ZERO,0,2","ZERO,0,256","FRAME,0","POLL,0,"};
      for(i=0;i<sizeof(bad)/sizeof(bad[0]);++i) CHECK(neck_replay_event_parse(bad[i],&b,&e)==NR_FORMAT); }
    replace_field(text,2,"4294967296",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_FORMAT);
    replace_field(text,12,"1e999",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_FORMAT);
    replace_field(text,12,"0x1p0",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_FORMAT);
    replace_field(text,12,"1e-999",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_FORMAT);
    replace_field(text,12,"NaN",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_FORMAT);
    replace_field(text,12,".125",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_OK);
    replace_field(text,12,"-1.2e+2",second); CHECK(neck_replay_event_parse(second,&b,&e)==NR_OK);
    CHECK(neck_replay_event_parse(NULL,&b,&e)==NR_ARGUMENT); CHECK(neck_replay_event_write(NULL,&a)==NR_ARGUMENT);
}
static void read_lines(void)
{
    FILE *f; unsigned long n=0; uint64_t hash=NR_FNV_OFFSET; NeckReplayError e={0,{0}}; char line[NR_LINE_MAX]; unsigned i;
    ++groups; f=tmpfile(); CHECK(f!=NULL); fputs("\xef\xbb\xbf" NECK_REPLAY_INPUT_HEADER "\r\nPOLL,0\r\nPOLL,1",f); rewind(f);
    CHECK(nr_header(f,NECK_REPLAY_INPUT_HEADER,&n,&hash,&e)==NR_OK); CHECK(n==1);
    CHECK(nr_read_line(f,line,&n,&hash,&e)==1 && !strcmp(line,"POLL,0")); CHECK(nr_read_line(f,line,&n,&hash,&e)==1 && !strcmp(line,"POLL,1"));
    CHECK(nr_read_line(f,line,&n,&hash,&e)==0); CHECK(n==3); CHECK(fclose(f)==0);
    f=tmpfile(); CHECK(f!=NULL); for(i=0;i<NR_LINE_MAX;++i) fputc('0',f); fputc('\n',f); rewind(f);
    CHECK(nr_read_line(f,line,&n,&hash,&e)==-1); CHECK(fclose(f)==0);
    f=tmpfile(); CHECK(f!=NULL); fputs("POLL,",f); fputc(0,f); fputs("0\n",f); rewind(f); CHECK(nr_read_line(f,line,&n,&hash,&e)==-1); CHECK(fclose(f)==0);
}
static void configs(void)
{
    NeckReplayConfig a,b; NeckReplayError e={0,{0}}; uint64_t hash; char p[1024],*data,*line; FILE *f;
    ++groups; neck_replay_default_config(&a); a.start_wall_us=UINT64_C(9223372036854775809);
    a.config_id=UINT32_MAX; a.world_frame_id=UINT32_MAX; a.parameters.posture.continuous_duration_us=1234567;
    f=new_test_file("cfg.csv"); CHECK(neck_replay_config_write(f,&a)==NR_OK); CHECK(fclose(f)==0); path(p,"cfg.csv");
    CHECK(neck_replay_config_read(p,&b,&hash,&e)==NR_OK); CHECK(b.start_wall_us==a.start_wall_us); CHECK(b.config_id==UINT32_MAX);
    CHECK(b.parameters.posture.continuous_duration_us==1234567); CHECK(b.parameters.pipeline.cutoff_hz==a.parameters.pipeline.cutoff_hz);
    f=new_test_file("cfg_roundtrip.csv"); CHECK(neck_replay_config_write(f,&b)==NR_OK); CHECK(fclose(f)==0); same_file("cfg.csv","cfg_roundtrip.csv");
    ++groups; data=all("cfg.csv");
    f=new_test_file("cfg_duplicate.csv"); fputs(data,f); fputs("config_id,1\n",f); CHECK(fclose(f)==0); path(p,"cfg_duplicate.csv"); CHECK(neck_replay_config_read(p,&b,&hash,&e)==NR_FORMAT);
    f=new_test_file("cfg_unknown.csv"); fputs(data,f); fputs("unknown,1\n",f); CHECK(fclose(f)==0); path(p,"cfg_unknown.csv"); CHECK(neck_replay_config_read(p,&b,&hash,&e)==NR_FORMAT);
    line=strrchr(data,'\n'); CHECK(line!=NULL); *line=0; line=strrchr(data,'\n'); CHECK(line!=NULL); line[1]=0;
    f=new_test_file("cfg_missing.csv"); fputs(data,f); CHECK(fclose(f)==0); path(p,"cfg_missing.csv"); CHECK(neck_replay_config_read(p,&b,&hash,&e)==NR_FORMAT); free(data);
    a.parameters.pipeline.cutoff_hz=0; f=new_test_file("cfg_invalid.csv"); CHECK(neck_replay_config_write(f,&a)==NR_OK); CHECK(fclose(f)==0); path(p,"cfg_invalid.csv"); CHECK(neck_replay_config_read(p,&b,&hash,&e)==NR_CONFIG);
}
static void truth_tests(void)
{
    NeckReplayTruth t; NeckReplayError e={0,{0}}; double out[2]; uint8_t mask; unsigned i; FILE *f; char p[1024]; NRTruths set;
    ++groups; CHECK(neck_replay_truth_parse("18446744073709551615,0,12,0,0,10,0,0,5,0,0",&t,&e)==NR_OK && t.time_us==UINT64_MAX);
    CHECK(neck_replay_truth_parse("0,1,12,0,0,10,0,0,5,0,0",&t,&e)==NR_FORMAT);
    CHECK(neck_replay_truth_parse("0,0,12,0,0,nan,0,0,5,0,0",&t,&e)==NR_FORMAT);
    CHECK(neck_replay_truth_parse("0,0,181,0,0,10,0,0,5,0,0",&t,&e)==NR_FORMAT);
    for(i=1;i<=80;++i) { neck_replay_ideal_proxy((double)i,0,out,&mask); CHECK(mask==3); CHECK(fabs(out[0]-(double)i)<1e-11); CHECK(out[1]==0); }
    neck_replay_ideal_proxy(0,15,out,&mask); CHECK(mask==2); CHECK(fabs(out[1]-15)<1e-12);
    neck_replay_ideal_proxy(0,0,out,&mask); CHECK(mask==3 && out[0]==0 && out[1]==0);
    neck_replay_ideal_proxy(100,0,out,&mask); CHECK(mask==0);
    neck_replay_ideal_proxy(30,20,out,&mask); CHECK(mask==3); CHECK(fabs(out[1]-17.229396562958897)<1e-10); CHECK(out[0]>30);
    f=new_test_file("truth_duplicate.csv"); fprintf(f,"%s\n0,0,12,0,0,10,0,0,5,0,0\n0,0,12,0,0,10,0,0,5,0,0\n",NECK_REPLAY_TRUTH_HEADER); CHECK(fclose(f)==0);
    path(p,"truth_duplicate.csv"); CHECK(nr_truths_read(p,&set,&e)==NR_FORMAT); CHECK(set.count==1); CHECK(nr_truth_at(&set,0,0)!=NULL); CHECK(nr_truth_at(&set,0,1)==NULL); CHECK(nr_truth_at(&set,1,0)==NULL); free(set.items);
}
static void fixtures(void)
{
    const char *names[4]={"baseline","silence_faults","timing_faults","motion"}; unsigned i; NeckReplayError e={0,{0}}; NeckReplayStats s; char p[1024],out[128];
    for(i=0;i<4;++i) {
        ++groups; path(p,names[i]); CHECK(neck_replay_generate(names[i],p,&e)==NR_OK); snprintf(out,sizeof(out),"result_%s",names[i]);
        s=run(names[i],out,1); CHECK(s.commands==1); CHECK(s.poll_calls==6501); CHECK(s.node_rows==3*s.outputs); CHECK(s.relative_rows==4*s.outputs);
        CHECK(s.input_frames==(i==1?6451:6501)); CHECK(s.outputs==(i==1?6499:6501)); CHECK(s.rejected_frames==(i==2?3:0));
        CHECK(s.timeout_edges==(i==1?1:0)); CHECK(s.poll_outputs==(i==1?48:0)); CHECK(s.triggers==(i==2?3:(i==3?0:1))); CHECK(s.clears==s.triggers);
    }
}
static void export_checks(const char *result)
{
    Csv c; char name[128]; uint64_t rows=0; unsigned i;
    static const struct { const char *key; uint32_t bit; } raw_fields[]={{"raw_pitch_deg",NECK_VALID_PITCH_RAW},{"raw_lateral_deg",NECK_VALID_LATERAL_RAW},{"raw_norm_m_s2",NECK_VALID_NORM}};
    static const struct { const char *key; uint32_t bit; } filtered_fields[]={{"filtered_absolute_pitch_deg",NECK_VALID_PITCH_RAW},{"filtered_absolute_lateral_deg",NECK_VALID_LATERAL_RAW},{"corrected_flexion_deg",NECK_VALID_FLEXION},{"corrected_right_lateral_deg",NECK_VALID_RIGHT_LATERAL},{"filtered_norm_m_s2",NECK_VALID_NORM},{"sample_time_us",NECK_VALID_SAMPLE_TIME},{"sample_age_us",NECK_VALID_SAMPLE_AGE}};
    snprintf(name,sizeof(name),"%s/nodes.csv",result); c=csv_open(name); csv_keys(&c);
    while(csv_next(&c)) {
        uint32_t r=(uint32_t)number(&c,"raw_mask"),f=(uint32_t)number(&c,"filtered_mask"); ++rows;
        CHECK(!strcmp(get(&c,"mode"),"OFFLINE_REPLAY"));
        for(i=0;i<sizeof(raw_fields)/sizeof(raw_fields[0]);++i) CHECK((get(&c,raw_fields[i].key)[0]!=0)==((r&raw_fields[i].bit)!=0));
        for(i=0;i<sizeof(filtered_fields)/sizeof(filtered_fields[0]);++i) CHECK((get(&c,filtered_fields[i].key)[0]!=0)==((f&filtered_fields[i].bit)!=0));
    }
    CHECK(rows>19000); CHECK(fclose(c.f)==0);
    snprintf(name,sizeof(name),"%s/relative.csv",result); c=csv_open(name); csv_keys(&c); rows=0;
    while(csv_next(&c)) {
        uint32_t m=(uint32_t)number(&c,"valid_mask"); ++rows;
        CHECK((get(&c,"raw_w")[0]!=0)==((m&NECK_REL_RAW_QUAT)!=0)); CHECK((get(&c,"raw_flexion_deg")[0]!=0)==((m&NECK_REL_RAW_EULER)!=0));
        CHECK((get(&c,"raw_rotation_deg")[0]!=0)==((m&NECK_REL_RAW_ANGLE)!=0)); CHECK((get(&c,"corrected_w")[0]!=0)==((m&NECK_REL_CORRECTED_QUAT)!=0));
        CHECK((get(&c,"corrected_flexion_deg")[0]!=0)==((m&NECK_REL_CORRECTED_EULER)!=0)); CHECK((get(&c,"corrected_rotation_deg")[0]!=0)==((m&NECK_REL_CORRECTED_ANGLE)!=0));
    }
    CHECK(rows>25000); CHECK(fclose(c.f)==0);
}
static void direct_replay(void)
{
    NeckProcessingContext ctx; NeckProcessingOutput o; NeckReplayConfig cfg; NeckReplayEvent ev; NeckReplayError e={0,{0}};
    FILE *f; Csv c; char p[1024],line[NR_LINE_MAX]; unsigned long n=0; uint64_t hash=NR_FNV_OFFSET,config_hash; uint64_t outputs=0;
    ++groups; path(p,"silence_faults/config.csv"); CHECK(neck_replay_config_read(p,&cfg,&config_hash,&e)==NR_OK);
    CHECK(neck_processing_init(&ctx,cfg.config_id,cfg.world_frame_id,(const double (*)[4])cfg.mounts,cfg.start_wall_us,&cfg.parameters)==NECK_OK);
    path(p,"silence_faults/inputs.csv"); f=fopen(p,"rb"); CHECK(f!=NULL); CHECK(nr_header(f,NECK_REPLAY_INPUT_HEADER,&n,&hash,&e)==NR_OK);
    c=csv_open("result_silence_faults/frames.csv"); csv_keys(&c);
    while(nr_read_line(f,line,&n,&hash,&e)==1) {
        NeckResult result=NECK_OK; int emit=0; CHECK(neck_replay_event_parse(line,&ev,&e)==NR_OK);
        if(ev.kind==NR_ZERO) CHECK(neck_processing_request_zero(&ctx,ev.now_us,ev.origin)==NECK_OK);
        else if(ev.kind==NR_POLL) emit=neck_processing_poll(&ctx,ev.now_us,&o);
        else if(ev.kind==NR_FRAME) { result=neck_processing_process(&ctx,&ev.frame,ev.now_us,&o); emit=1; }
        if(!emit) continue;
        CHECK(csv_next(&c)); CHECK(number(&c,"output_id")==outputs++); CHECK(number(&c,"result")==result);
        CHECK(number(&c,"sequence")==o.pose.quality.sequence); CHECK(number(&c,"decision_time_us")==o.pose.quality.decision_time_us);
        CHECK(number(&c,"quality_flags")==o.pose.quality.flags); CHECK(number(&c,"accel_mask")==o.pose.quality.accel_usable_mask);
        CHECK(number(&c,"quat_mask")==o.pose.quality.orientation_usable_mask); CHECK(number(&c,"flex_valid")==o.pose.posture.flexion.valid);
        CHECK(number(&c,"flex_alarm")==o.pose.posture.flexion.alarm_level); CHECK(number(&c,"flex_events")==o.pose.posture.flexion.events);
        CHECK(number(&c,"flex_warning_us")==o.pose.posture.flexion.warning_elapsed_us); CHECK(number(&c,"flex_danger_us")==o.pose.posture.flexion.danger_elapsed_us);
        CHECK(number(&c,"epoch")==o.pose.invalidation_epoch); CHECK(number(&c,"valid_until_us")==o.pose.valid_until_us);
    }
    CHECK(outputs==6499); CHECK(!csv_next(&c)); CHECK(fclose(c.f)==0); CHECK(fclose(f)==0);
}
static void deterministic(void)
{
    static const char *const names[]={"frames.csv","nodes.csv","relative.csv","events.csv","commands.csv","config_used.csv","comparison.csv","metadata.txt","COMPLETE.txt","index.html","raw_filtered.svg","corrected.svg","levels.svg","timers.svg","quality.svg","relative_0.svg","relative_1.svg","relative_2.svg","relative_3.svg","construction.svg"};
    unsigned i; char a[128],b[128],p[1024]; NeckReplayStats s; NeckReplayError e={0,{0}};
    ++groups; (void)run("silence_faults","result_identical",1);
    for(i=0;i<sizeof(names)/sizeof(names[0]);++i) { snprintf(a,sizeof(a),"result_silence_faults/%s",names[i]); snprintf(b,sizeof(b),"result_identical/%s",names[i]); same_file(a,b); }
    path(p,"silence_faults"); CHECK(neck_replay_generate("silence_faults",p,&e)==NR_EXISTS); CHECK(neck_replay_generate("no_such_scenario",p,&e)==NR_ARGUMENT);
    { char in[1024],cf[1024]; path(in,"silence_faults/inputs.csv"); path(cf,"silence_faults/config.csv"); path(p,"result_identical"); CHECK(neck_replay_run(in,cf,NULL,p,&s,&e)==NR_EXISTS); }
    ++groups; (void)run("silence_faults","result_no_truth",0);
    for(i=0;i<6;++i) { snprintf(a,sizeof(a),"result_silence_faults/%s",names[i]); snprintf(b,sizeof(b),"result_no_truth/%s",names[i]); same_file(a,b); }
    CHECK(!exists("result_no_truth/comparison.csv")); CHECK(!exists("result_no_truth/construction.svg"));
    path(p,"regenerated"); CHECK(neck_replay_generate("silence_faults",p,&e)==NR_OK);
    same_file("silence_faults/inputs.csv","regenerated/inputs.csv"); same_file("silence_faults/config.csv","regenerated/config.csv"); same_file("silence_faults/truth.csv","regenerated/truth.csv");
}
static void tiny_input(const char *dir,const NeckReplayConfig *cfg,const NeckReplayEvent *events,unsigned count)
{
    char p[1024],n[128]; FILE *f; unsigned i; NeckReplayError e={0,{0}};
    path(p,dir); CHECK(nr_new_directory(p,&e)==NR_OK); snprintf(n,sizeof(n),"%s/config.csv",dir); f=new_test_file(n); CHECK(neck_replay_config_write(f,cfg)==NR_OK); CHECK(fclose(f)==0);
    snprintf(n,sizeof(n),"%s/inputs.csv",dir); f=new_test_file(n); fprintf(f,"%s\n",NECK_REPLAY_INPUT_HEADER);
    for(i=0;i<count;++i) { CHECK(neck_replay_event_write(f,&events[i])==NR_OK); }
    CHECK(fclose(f)==0);
}
static void boundaries(void)
{
    NeckReplayConfig cfg; NeckReplayEvent ev[5]; NeckReplayStats s; Csv c; char *data;
    ++groups; neck_replay_default_config(&cfg); memset(ev,0,sizeof(ev));
    ev[0].kind=NR_FRAME; neck_simulator_frame(&ev[0].frame,0,0);
    ev[1].kind=NR_POLL; ev[1].now_us=30000; ev[2].kind=NR_POLL; ev[2].now_us=30001; ev[3].kind=NR_POLL; ev[3].now_us=40000;
    tiny_input("boundary",&cfg,ev,4); s=run("boundary","result_boundary",0); CHECK(s.outputs==3); CHECK(s.poll_outputs==2); CHECK(s.timeout_edges==1); CHECK(s.commands==0);
    c=csv_open("result_boundary/frames.csv"); csv_keys(&c); CHECK(csv_next(&c)); CHECK(number(&c,"zero_request")==0); CHECK(number(&c,"posture_reference")==0);
    CHECK(csv_next(&c)); CHECK(number(&c,"decision_time_us")==30001); CHECK(number(&c,"quality_flags")&NECK_Q_TIMEOUT_EDGE);
    CHECK(csv_next(&c)); CHECK(number(&c,"decision_time_us")==40000); CHECK(number(&c,"flex_events")==0); CHECK(!csv_next(&c)); CHECK(fclose(c.f)==0);
    cfg.start_wall_us=UINT64_C(9223372036854775809); cfg.parameters.quality.recovery_us=0;
    ev[0].now_us=cfg.start_wall_us; ev[0].frame.acceleration.frame_time_us=ev[0].frame.orientation.frame_time_us=cfg.start_wall_us;
    { unsigned i; for(i=0;i<3;++i) ev[0].frame.acceleration.nodes[i].sample_time_us=ev[0].frame.orientation.nodes[i].sample_time_us=cfg.start_wall_us; }
    ev[1].kind=NR_FRAME; ev[1].now_us=cfg.start_wall_us+1; ev[1].frame=ev[0].frame; ev[1].frame.acceleration.sequence=ev[1].frame.orientation.sequence=1;
    ev[1].frame.acceleration.frame_time_us=ev[1].frame.orientation.frame_time_us=ev[1].now_us;
    { unsigned i; for(i=0;i<3;++i) ev[1].frame.acceleration.nodes[i].sample_time_us=ev[1].frame.orientation.nodes[i].sample_time_us=ev[1].now_us; }
    tiny_input("large_time",&cfg,ev,2); s=run("large_time","result_large_time",0); CHECK(s.outputs==2 && s.rejected_frames==0);
    c=csv_open("result_large_time/frames.csv"); csv_keys(&c); CHECK(csv_next(&c)); CHECK(number(&c,"decision_time_us")==cfg.start_wall_us); CHECK(csv_next(&c)); CHECK(number(&c,"decision_time_us")==cfg.start_wall_us+1); CHECK(fclose(c.f)==0);
    data=all("result_large_time/raw_filtered.svg"); CHECK(strstr(data,"M72.000")!=NULL); CHECK(strstr(data,"L1120.000")!=NULL); free(data);
    ++groups; neck_replay_default_config(&cfg); cfg.parameters.quality.no_frame_timeout_us=50000;
    memset(ev,0,sizeof(ev)); ev[0].kind=NR_POLL; ev[0].now_us=30001; ev[1].kind=NR_POLL; ev[1].now_us=50000; ev[2].kind=NR_POLL; ev[2].now_us=50001;
    tiny_input("custom_timeout",&cfg,ev,3); s=run("custom_timeout","result_custom_timeout",0); CHECK(s.input_frames==0 && s.outputs==1 && s.timeout_edges==1);
    c=csv_open("result_custom_timeout/frames.csv"); csv_keys(&c); CHECK(csv_next(&c)); CHECK(number(&c,"decision_time_us")==50001); CHECK(fclose(c.f)==0);
}
static void commands_and_hardware(void)
{
    NeckReplayConfig cfg; NeckReplayEvent ev[5]; NeckReplayStats s; Csv c; unsigned i;
    ++groups; neck_replay_default_config(&cfg); cfg.parameters.quality.recovery_us=0; memset(ev,0,sizeof(ev));
    ev[0].kind=NR_ZERO; ev[1].kind=NR_CANCEL; ev[2].kind=NR_FRAME; neck_simulator_frame(&ev[2].frame,0,0);
    ev[3].kind=NR_RESET; ev[3].now_us=10000; ev[4].kind=NR_POLL; ev[4].now_us=40001;
    tiny_input("commands",&cfg,ev,5); s=run("commands","result_commands",0); CHECK(s.commands==3 && s.outputs==3 && s.timeout_edges==1);
    c=csv_open("result_commands/commands.csv"); csv_keys(&c); CHECK(csv_next(&c)); CHECK(number(&c,"request_state")==NECK_REQUEST_WAIT_QUALITY);
    CHECK(csv_next(&c)); CHECK(number(&c,"request_state")==NECK_REQUEST_IDLE); CHECK(csv_next(&c)); CHECK(!strcmp(get(&c,"command"),"RESET")); CHECK(fclose(c.f)==0);
    neck_replay_default_config(&cfg); cfg.parameters.quality.recovery_us=0; memset(ev,0,sizeof(ev)); ev[0].kind=NR_FRAME; neck_simulator_frame(&ev[0].frame,0,0);
    ev[0].frame.acceleration.source=ev[0].frame.orientation.source=NECK_SOURCE_HARDWARE;
    for(i=0;i<3;++i) ev[0].frame.acceleration.nodes[i].calibration_accel=ev[0].frame.orientation.nodes[i].calibration_system=3;
    tiny_input("hardware_label",&cfg,ev,1);
    { FILE *f=new_test_file("hardware_label/truth.csv"); fprintf(f,"%s\n0,0,12,0,0,10,0,0,5,0,0\n",NECK_REPLAY_TRUTH_HEADER); CHECK(fclose(f)==0); }
    s=run("hardware_label","result_hardware_label",1); CHECK(s.outputs==1);
    c=csv_open("result_hardware_label/comparison.csv"); csv_keys(&c); CHECK(csv_next(&c)); CHECK(number(&c,"source")==1); CHECK(number(&c,"construction_joined")==0); CHECK(!get(&c,"body_pitch_deg")[0]); CHECK(fclose(c.f)==0);
}
static void failure_and_paths(void)
{
    NeckReplayConfig cfg; NeckReplayEvent ev; char in[1024],cf[1024],dir[1024]; FILE *f; NeckReplayStats s; NeckReplayError e={0,{0}};
    ++groups; neck_replay_default_config(&cfg); memset(&ev,0,sizeof(ev)); ev.kind=NR_FRAME; neck_simulator_frame(&ev.frame,0,0); tiny_input("malformed_late",&cfg,&ev,1);
    path(in,"malformed_late/inputs.csv"); f=fopen(in,"ab"); CHECK(f!=NULL); fputs("POLL,-1\n",f); CHECK(fclose(f)==0);
    path(cf,"malformed_late/config.csv"); path(dir,"result_malformed_late"); CHECK(neck_replay_run(in,cf,NULL,dir,&s,&e)==NR_FORMAT);
    CHECK(e.line==3); CHECK(exists("result_malformed_late/FAILED.txt")); CHECK(exists("result_malformed_late/frames.csv"));
    CHECK(!exists("result_malformed_late/COMPLETE.txt")); CHECK(!exists("result_malformed_late/index.html"));
    path(dir,"result_missing_input"); CHECK(neck_replay_run("__no_file__",cf,NULL,dir,&s,&e)==NR_IO); CHECK(exists("result_missing_input/FAILED.txt"));
    CHECK(neck_replay_run(NULL,cf,NULL,dir,&s,&e)==NR_ARGUMENT); CHECK(nr_new_directory(NULL,&e)==NR_ARGUMENT);
    CHECK(nr_path(in,NULL,"a",&e)==NR_ARGUMENT); CHECK(nr_close(NULL,&e)==NR_OK);
}
static void known_faults_and_svg(void)
{
    Csv c; char *svg,*d,*end; unsigned moves=0; int hit_timeout=0,hit_bad=0,hit_quat=0;
    ++groups; c=csv_open("result_silence_faults/frames.csv"); csv_keys(&c);
    while(csv_next(&c)) {
        uint64_t time=number(&c,"decision_time_us");
        if(time==32030000) { CHECK(number(&c,"flex_events")==10); CHECK(number(&c,"flex_alarm")==0); hit_timeout=1; }
        if(time==36000000) { CHECK(number(&c,"accel_mask")==5); CHECK(number(&c,"quat_mask")==5); CHECK(number(&c,"flex_valid")==0); hit_bad=1; }
        if(time==38000000) { CHECK(number(&c,"accel_mask")==7); CHECK(number(&c,"quat_mask")==6); CHECK(number(&c,"flex_valid")==1); hit_quat=1; }
    }
    CHECK(hit_timeout && hit_bad && hit_quat); CHECK(fclose(c.f)==0);
    svg=all("result_silence_faults/corrected.svg"); d=strstr(svg,"data-series=\"corrected flexion\""); CHECK(d!=NULL); d=strstr(d," d=\""); CHECK(d!=NULL); d+=4; end=strchr(d,'"'); CHECK(end!=NULL);
    while(d<end) if(*d++=='M') ++moves;
    CHECK(moves>=3); CHECK(strstr(svg,"nan")==NULL); CHECK(strstr(svg,"Infinity")==NULL); free(svg);
    c=csv_open("result_motion/nodes.csv"); csv_keys(&c); hit_bad=0;
    while(csv_next(&c)) if(number(&c,"decision_time_us")==20000000 && number(&c,"node")==1) { CHECK(number(&c,"raw_mask")&NECK_VALID_PITCH_RAW); CHECK(fabs(strtod(get(&c,"raw_pitch_deg"),NULL)-36.86989764584402)<1e-10); hit_bad=1; }
    CHECK(hit_bad); CHECK(fclose(c.f)==0);
    puts("S08 limitation proved: same-norm wrong-direction acceleration passes software gate; NOT physical confidence.");
}
int main(int argc,char **argv)
{
    NeckReplayError e={0,{0}}; (void)setlocale(LC_NUMERIC,"C"); CHECK(argc==2); CHECK(strlen(argv[1])<sizeof(root)); strcpy(root,argv[1]); CHECK(nr_new_directory(root,&e)==NR_OK);
    codecs(); read_lines(); configs(); truth_tests(); fixtures();
    ++groups; export_checks("result_baseline"); export_checks("result_silence_faults"); export_checks("result_timing_faults"); export_checks("result_motion");
    direct_replay(); deterministic(); boundaries(); commands_and_hardware(); failure_and_paths(); known_faults_and_svg();
    printf("S08 C replay: %u groups, %u assertions passed.\n",groups,checks); return 0;
}
