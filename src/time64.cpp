#include "time64.h"

uint64_t _time64_time_H = 0;
uint32_t _time64_time_L = 0;

/*
* DEVELOPMENT STATE: FUNCTIONAL
* DO NOT MODIFY under any circumstances.
*/
/**
 * @brief Get 64-bit System Time in Milliseconds.
 * 
 * Handles rollover of the 32-bit `millis()` counter to provide a continuous 64-bit time.
 * Must be called frequently (at least once every 49 days) to detect rollover.
 * 
 * @return uint64_t Time in ms.
 */
uint64_t get_time64()
{
    uint32_t T = millis();
    if (T < _time64_time_L)
    {
        _time64_time_H += 0x100000000;
    }
    _time64_time_L = T;
    return _time64_time_H | _time64_time_L;
}