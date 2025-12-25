#pragma once

#include "CommandTypes.h"

namespace CommandRouter {
    void Init(bool isKlipper);
    
    // Main loop function to poll and process
    void Run();
    
    // Core routing function
    void Route(UnifiedCommandType type, uint8_t* buffer, uint16_t length);

    // Helper to send packets (This might need abstraction later, but keeping for now)
    void SendPacket(uint8_t *data, uint16_t length);
}
