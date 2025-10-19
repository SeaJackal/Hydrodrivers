#ifndef HYDRV_CLOCK_H_
#define HYDRV_CLOCK_H_

#include "hydrv_common.h"

#define HYDRV_NO_TIMEOUT UINT32_MAX

#define HYDRV_CLOCK_TIMEOUT_MS 1000

typedef union
{
    uint8_t summary;
    struct
    {
        int default_tick : 1;
        int hsi : 1;
        int hse : 1;
        int pll : 1;
        int sys_tick : 1;
    };
} hydrv_ClockStatus;

extern hydrv_ClockStatus hydrv_clock_status;

hydrv_ReturnCode hydrv_Clock_ConfigureHSI(void);

void hydrv_Clock_SysTickHandler(void);

uint32_t hydrv_Clock_GetSystemTime(void);
void hydrv_Clock_Delay(uint32_t time_ms);
hydrv_ReturnCode hydrv_Clock_WaitUntilTrue(hydrv_ConditionFunc *condition, uint32_t timeout);

#endif
