#pragma once

#include "BambuBusProtocol.h"

// Forward declaration of ControlLogic functions or class
// For now, we assume global functions or a static class from ControlLogic.h

namespace CommandRouter {
    void Init();
    
    // Main loop function to poll and process
    void Run();
    
    // Helper to send packets (using Hardware TX)
    void SendPacket(uint8_t *data, uint16_t length);
}
