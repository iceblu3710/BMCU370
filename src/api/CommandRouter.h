#pragma once

#include "APIBase.h"

class I_MMU_Transport;

class CommandRouter : public APIBase {
public:
    CommandRouter();
    virtual ~CommandRouter() {}

    /**
     * @brief Initialize the Command Router and API Layer.
     * 
     * @param logic Pointer to Logic Instance.
     * @param transport Pointer to transport layer.
     */
    void Init(MMU_Logic* logic, I_MMU_Transport* transport) override;
    
    /**
     * @brief Main processing loop.
     */
    void Run() override;

private:
    MMU_Logic* _mmu;
    I_MMU_Transport* _transport;
};
