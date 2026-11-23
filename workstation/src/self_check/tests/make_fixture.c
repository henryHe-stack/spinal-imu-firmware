/* Generates OFFLINE TEST input using the same formatter as firmware.
 * The replay flag in the receiver must remain set; this is NOT hardware data. */
#include "diag_stats.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc,char **argv)
{
    DiagReport r;DiagCycle c;unsigned i,j;uint64_t at=0;char output[1500];
    const char *scenario=argc>1?argv[1]:"good";
    uint8_t mask=strcmp(scenario,"single")==0?2u:7u;
    memset(&r,0,sizeof(r));
    if(!diag_stats_init(&r.stats,mask)) return 1;
    r.free_heap_bytes=4096;r.acquire_stack_words=192;r.report_stack_words=200;
    if(!diag_format_header(mask,1,0,output,sizeof(output))) return 2;
    fputs(output,stdout);
    for(i=0;i<300;++i) {
        uint64_t cursor=at;memset(&c,0,sizeof(c));c.start_us=at;
        for(j=0;j<3;++j) if(mask & (1u<<j)) {
            DiagNodeRead *n=&c.nodes[j];n->start_us=cursor;n->end_us=cursor+1000;
            cursor+=1200;n->quaternion[0]=1;n->accel[2]=9.81;n->observed=1;
            n->calibration_system=n->calibration_accel=3;
            if(j==1 && strcmp(scenario,"readfail")==0 && i==150) n->result=2;
            if(j==1 && strcmp(scenario,"invalid")==0 && i==150) n->quaternion[0]=0;
            if(j==1 && strcmp(scenario,"cal")==0) n->calibration_system=1;
        }
        c.end_us=cursor;
        if(strcmp(scenario,"clock")==0 && i==150) c.end_us=at-1;
        (void)diag_stats_add(&r.stats,&c);
        if((i+1)%100==0) {
            if(!diag_format_report(&r,output,sizeof(output))) return 3;
            fputs(output,stdout);
        }
        at+=(strcmp(scenario,"timing")==0 && i==149)?13000:10000;
    }
    return 0;
}
