#ifndef HYDRV_COMMON_H_
#define HYDRV_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#define HYDRV_MACRO_CAT(first, second) first

typedef enum
{
    HYDRV_OK = 0,
    HYDRV_FAIL,
    HYDRV_TIMEOUT,
    HYDRV_BUSY,
    HYDRV_NO_DATA
} hydrv_ReturnCode;

typedef bool hydrv_ConditionFunc();

#endif
