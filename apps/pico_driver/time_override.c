#include "FreeRTOS.h"
#include "task.h"
#include <time.h>
#include <stdint.h>

uint64_t rmw_uros_epoch_nanos()
{
    return ((uint64_t)xTaskGetTickCount() * 1000000ULL); // 1 ms tick → ns
}

uint64_t rmw_uros_epoch_millis()
{
    return ((uint64_t)xTaskGetTickCount() * 1000ULL); // 1 ms tick → ns
}

int clock_gettimea(int clk_id, struct timespec *tp) {
    if(tp) {
        tp->tv_sec = 0;
        tp->tv_nsec = 0;
    }
    return 0;
}

// Dummy clock_gettime() for compatibility
int clock_gettime(int clk_id, struct timespec *tp)
{
    (void)clk_id;
    if (tp)
    {
        tp->tv_sec = 0;
        tp->tv_nsec = (long)(xTaskGetTickCount() * 1000000ULL);
    }
    return 0;
}