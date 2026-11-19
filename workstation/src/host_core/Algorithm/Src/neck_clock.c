#include "neck_clock.h"
#include <stddef.h>
#include <string.h>
NeckResult neck_clock_init(NeckTickClock *c, uint32_t tick, uint32_t period)
{
    if(c==NULL) return NECK_BAD_ARGUMENT;
    memset(c,0,sizeof(*c));
    if(period==0) return NECK_BAD_CONFIG;
    c->previous_tick=tick; c->tick_period_us=period; c->initialized=1; return NECK_OK;
}
NeckResult neck_clock_update(NeckTickClock *c, uint32_t tick, uint64_t *time)
{
    uint64_t dt;
    if(c==NULL || time==NULL) return NECK_BAD_ARGUMENT;
    if(!c->initialized) return NECK_BAD_CONFIG;
    *time=c->elapsed_us;
    if(c->fault) return NECK_BAD_FRAME;
    /* 先按 32 位无符号数求节拍差，再扩为 64 位换算微秒，可处理计数器回绕。 */
    /* 例如节拍从 0xFFFFFFFE 回绕到 1，差值为 3，累计时间增加 3 个节拍周期。 */
    dt=(uint64_t)(uint32_t)(tick-c->previous_tick)*c->tick_period_us;
    /* 累计时间溢出时锁存故障，后续调用保留最后一次有效时间。 */
    if(dt>UINT64_MAX-c->elapsed_us) { c->fault=1; return NECK_BAD_FRAME; }
    c->elapsed_us+=dt; c->previous_tick=tick; *time=c->elapsed_us; return NECK_OK;
}
