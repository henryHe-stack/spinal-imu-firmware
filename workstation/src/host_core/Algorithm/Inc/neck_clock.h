/* 将 32 位节拍累计为微秒时间；初始化时的节拍对应 0 微秒。 */
#ifndef NECK_CLOCK_H
#define NECK_CLOCK_H
#include "neck_tilt.h"
typedef struct {
    uint64_t elapsed_us;
    uint32_t previous_tick, tick_period_us;
    uint8_t initialized, fault;
} NeckTickClock;
NeckResult neck_clock_init(NeckTickClock *clock, uint32_t tick, uint32_t tick_period_us);
/* Caller serializes access. At least one call per 32-bit tick wrap is required.
 * uint64 overflow latches a fault; never silently wraps the wall epoch. */
/* 调用方须串行更新，且相邻调用间隔应小于一个完整回绕周期，否则仅凭两个读数无法还原经过的节拍数。 */
NeckResult neck_clock_update(NeckTickClock *clock, uint32_t tick, uint64_t *time_us);
#endif
