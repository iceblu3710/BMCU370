#pragma once

#include <stdint.h>
#include "BambuBusProtocol.h"

// --- Restored Enums ---
enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
};

enum class filament_motion_enum
{
    stop,
    send,
    pull,
    slow_send,
    pressure_ctrl_idle,
    pressure_ctrl_in_use, 
    pressure_ctrl_on_use // Kept for compatibility if code uses it, but will fix usages to in_use where possible
};

namespace ControlLogic {

    void Init();
    void Run();
    
    // Connectivity
    void UpdateConnectivity(bool online);
    
    // Command Processing (Short)
    void ProcessMotionShort(uint8_t* buffer, uint16_t length);
    void ProcessMotionLong(uint8_t* buffer, uint16_t length);
    void ProcessOnlineDetect(uint8_t* buffer, uint16_t length);
    void ProcessREQx6(uint8_t* buffer, uint16_t length);
    void ProcessSetFilamentInfo(uint8_t* buffer, uint16_t length);
    void ProcessHeartbeat(uint8_t* buffer, uint16_t length);
    
    // Command Processing (Long)
    void ProcessLongPacket(struct long_packge_data &data);
    
    // State Accessors (if needed by other modules, or for debugging)
    uint16_t GetDeviceType();

    // --- Restored Functions ---
    void SaveSettings();
    void SetNeedToSave();
}

