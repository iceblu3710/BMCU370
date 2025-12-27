#pragma once

#include <stdint.h>

class MMU_Logic;
class I_MMU_Transport;

/**
 * @brief Abstract Base Class for API implementations.
 * Allows decoupling of the main loop/Logic from the specific communication method.
 */
class APIBase {
public:
    virtual ~APIBase() {}

    /**
     * @brief Initialize the API Layer.
     * @param logic Pointer to the main business logic instance.
     * @param transport Pointer to the transport layer for communication.
     */
    virtual void Init(MMU_Logic* logic, I_MMU_Transport* transport) = 0;

    /**
     * @brief Main processing loop. Should be called frequently.
     */
    virtual void Run() = 0;
};
