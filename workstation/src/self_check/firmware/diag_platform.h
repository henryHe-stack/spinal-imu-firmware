#ifndef DIAG_PLATFORM_H
#define DIAG_PLATFORM_H
#include "main.h"
#include <stdint.h>
int diag_timer_start(void);
uint64_t diag_now_us(void);
void diag_marker(int high);
void diag_start(void);
extern volatile uint32_t g_diag_fatal;
#endif
