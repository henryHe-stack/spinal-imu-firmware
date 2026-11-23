#include "neck_replay.h"
#include <locale.h>
#include <string.h>
static void help(void)
{
    puts("S08 C-only offline replay (NOT MCU firmware, NOT hardware validation)\n"
         "neck_replay generate <baseline|silence_faults|timing_faults|motion> <NEW_DIRECTORY>\n"
         "neck_replay replay <inputs.csv> <config.csv> <NEW_RESULT_DIRECTORY> [truth.csv]\n"
         "No overwrites. No implicit poll/zero. Optional truth is presentation only.\n"
         "Typed CSV v1: see HostTools/README.md. Numeric locale: C.");
}
int main(int argc,char **argv)
{
    NeckReplayError e={0,{0}}; NeckReplayStats stats; int result;
    (void)setlocale(LC_NUMERIC,"C");
    if(argc==2 && !strcmp(argv[1],"--help")) { help(); return 0; }
    if(argc==4 && !strcmp(argv[1],"generate")) result=neck_replay_generate(argv[2],argv[3],&e);
    else if((argc==5 || argc==6) && !strcmp(argv[1],"replay")) {
        result=neck_replay_run(argv[2],argv[3],argc==6?argv[5]:NULL,argv[4],&stats,&e);
        if(result==NR_OK) printf("OFFLINE_REPLAY COMPLETE: frames=%llu polls=%llu outputs=%llu rejected=%llu triggers=%llu clears=%llu timeout_edges=%llu\n",
            (unsigned long long)stats.input_frames,(unsigned long long)stats.poll_calls,(unsigned long long)stats.outputs,
            (unsigned long long)stats.rejected_frames,(unsigned long long)stats.triggers,(unsigned long long)stats.clears,(unsigned long long)stats.timeout_edges);
    } else { help(); return NR_ARGUMENT; }
    if(result!=NR_OK) fprintf(stderr,"Replay status=%d line=%lu: %s\n",result,e.line,e.message);
    return result;
}
