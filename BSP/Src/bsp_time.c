#include "bsp_time.h"

#include "FreeRTOS.h"
#include "task.h"

AppStatus BspTime_Init(BspTime *self, TIM_HandleTypeDef *timer)
{
    if (self == NULL)
    {
        return APP_STATUS_INVALID_ARGUMENT;
    }

    self->timer = timer;
    self->high_resolution_ready = false;

    /*
     * TODO(TIMEBASE owner): configure TIM2 as a 1 MHz free-running counter and
     * extend its 16-bit counter in the update ISR before setting this flag.
     * The current CubeMX project still generates TIM2 with Prescaler = 0.
     */
    return APP_STATUS_OK;
}

uint32_t BspTime_NowUs(const BspTime *self)
{
    (void)self;

    /* Coarse fallback keeps the framework operational until TIM2 is finalized. */
    return ((uint32_t)xTaskGetTickCount()) * 1000U;
}

bool BspTime_IsHighResolution(const BspTime *self)
{
    return (self != NULL) && self->high_resolution_ready;
}
