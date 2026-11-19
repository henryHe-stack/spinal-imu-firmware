#include "neck_owner_step_example.h"
#include <stddef.h>
NeckResult neck_owner_step_example(NeckProcessingContext *context,
    const NeckAcquisitionFrame *frame, uint64_t now, NeckProcessingOutput *scratch,
    NeckOwnerPublish publish, void *user, unsigned *count)
{
    NeckResult result=NECK_OK; unsigned emitted=0;
    if(count!=NULL) *count=0;
    if(context==NULL || scratch==NULL || publish==NULL) return NECK_BAD_ARGUMENT;
    if(!context->initialized) return NECK_BAD_CONFIG;
    if(neck_processing_poll(context,now,scratch)) {
        publish(scratch,NECK_OWNER_POLL,NECK_OK,user); ++emitted;
    }
    if(frame!=NULL) {
        result=neck_processing_process(context,frame,now,scratch);
        publish(scratch,NECK_OWNER_FRAME,result,user); ++emitted;
    }
    if(count!=NULL) *count=emitted;
    return result;
}
