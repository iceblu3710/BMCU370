#pragma once

#include <stdint.h>
#include "BambuBusProtocol.h"

namespace ControlLogic {

    void Init();
    
    // Main loop logic
    void Run();
    
    // Interaction methods called by CommandRouter
    void UpdateConnectivity(bool online);
    void ProcessMotionShort(uint8_t* buffer, uint16_t length);
    void ProcessMotionLong(uint8_t* buffer, uint16_t length);
    void ProcessOnlineDetect(uint8_t* buffer, uint16_t length);
    void ProcessREQx6(uint8_t* buffer, uint16_t length);
    void ProcessSetFilamentInfo(uint8_t* buffer, uint16_t length);
    void ProcessHeartbeat(uint8_t* buffer, uint16_t length);
    void ProcessLongPacket(long_packge_data &data);
    
    // State Accessors (if needed by other modules, or for debugging)
    uint16_t GetDeviceType();
    
}
