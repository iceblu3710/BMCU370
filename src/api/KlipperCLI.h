#pragma once

#include <stdint.h>
class MMU_Logic;
class I_MMU_Transport;

/*
* DEVELOPMENT STATE: TESTING
* This namespace handles the Klipper JSON protocol which is currently in a testing state.
*/
namespace KlipperCLI {
    
    /**
     * @brief Initialize the CLI with logic and transport dependencies.
     * @param mmu Pointer to MMU_Logic instance.
     * @param transport Pointer to transport layer for communication.
     */
    void Init(MMU_Logic* mmu, I_MMU_Transport* transport);

    // Main loop processor - handles incoming serial data
    void Run();

    // Check if connected
    bool IsConnected();
}
