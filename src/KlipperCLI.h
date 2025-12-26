#pragma once

#include <stdint.h>

/*
* DEVELOPMENT STATE: TESTING
* This namespace handles the Klipper JSON protocol which is currently in a testing state.
*/
namespace KlipperCLI {
    
    // Initial greeting and setup
    void Init();

    // Main loop processor - handles incoming serial data
    void Run();

    // Check if connected
    bool IsConnected();

    // Feed a byte to the internal buffer
    void FeedByte(uint8_t byte);
}
