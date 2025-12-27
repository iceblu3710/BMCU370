#include "CommandRouter.h"
#include "KlipperCLI.h"
#include "I_MMU_Transport.h"
#include "MMU_Logic.h"

CommandRouter::CommandRouter() : _mmu(nullptr), _transport(nullptr) {
}

void CommandRouter::Init(MMU_Logic* mmu, I_MMU_Transport* transport) {
    _mmu = mmu;
    _transport = transport;
    
    // Initialize KlipperCLI with transport
    KlipperCLI::Init(mmu, transport);
}

void CommandRouter::Run() {
    KlipperCLI::Run();
}
