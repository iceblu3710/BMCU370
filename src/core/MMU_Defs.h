#pragma once

#include <stdint.h>

// Filament Status
enum class AMS_filament_status
{
    offline,
    online,
    NFC_waiting
};

// Filament Motion State
enum class AMS_filament_motion
{
    before_pull_back, // 0
    need_pull_back,   // 1
    need_send_out,    // 2
    in_use,           // 3
    idle              // 4
};

// Device Types (kept for compatibility with flash struct if needed, or remove later)
enum BambuBus_device_type
{
    BambuBus_none = 0x0000,
    BambuBus_AMS = 0x0700,
    BambuBus_AMS_lite = 0x1200,
};
