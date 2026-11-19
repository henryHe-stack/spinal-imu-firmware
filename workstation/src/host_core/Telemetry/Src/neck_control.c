#include "neck_control.h"
#include <string.h>
void neck_control_init(NeckControl *c,uint32_t boot,uint32_t session,uint32_t config)
{
    if(!c) return;
    memset(c,0,sizeof(*c)); c->boot=boot; c->session=session; c->config=config;
}
int neck_control_dispatch(NeckControl *c,const NeckWireMessage *m,uint64_t now,
    NeckCommandHandler handler,void *user,NeckWireControl *reply)
{
    const NeckWireControl *q; int ordered;
    if(!c || !m || !reply || !handler || m->header.type!=NECK_WIRE_COMMAND ||
        !m->body.control.request_id || m->body.control.result) return 0;
    q=&m->body.control; memset(reply,0,sizeof(*reply));
    reply->request_id=q->request_id; reply->opcode=q->opcode;
    if(!c->boot || m->header.boot_id!=c->boot || m->header.session_id!=c->session ||
        m->header.config_id!=c->config) { reply->result=NECK_CMD_IDENTITY; return 1; }
    ordered=q->opcode!=NECK_CMD_QUERY && q->opcode!=NECK_CMD_EVENT_ACK;
    if(ordered && q->request_id==c->high_water) {
        if(memcmp(q,&c->last_command,sizeof(*q))==0) *reply=c->last_reply;
        else reply->result=NECK_CMD_CONFLICT;
        return 1;
    }
    if(ordered && q->request_id<c->high_water) { reply->result=NECK_CMD_REPLAY; return 1; }
    if(now<m->header.emit_time_us || now-m->header.emit_time_us>2000000) {
        reply->result=NECK_CMD_EXPIRED; return 1;
    }
    reply->result=handler(user,q,reply->args);
    if(q->opcode==NECK_CMD_QUERY) reply->args[0]=c->high_water;
    if(ordered) { c->high_water=q->request_id; c->last_command=*q; c->last_reply=*reply; }
    return 1;
}
