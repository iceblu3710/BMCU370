#pragma once

#include <stdint.h>

enum class UnifiedCommandType {
    None,
    FilamentMotionShort,
    FilamentMotionLong,
    OnlineDetect,
    REQx6,
    SetFilamentInfo,
    Heartbeat,
    MCOnline,
    ReadFilamentInfo,
    SetFilamentInfoType2,
    Version,
    SerialNumber,
    NFCDetect,
    Error,
    ETC
};
