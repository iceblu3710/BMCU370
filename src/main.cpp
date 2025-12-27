#include <Arduino.h>
#include "BMCU_Hardware.h"
#include "UART_Transport.h"
#include "MMU_Logic.h"
#include "CommandRouter.h"


// Instance Management (Global scope to persist)
static BMCU_Hardware* hal = nullptr;
static UART_Transport* transport = nullptr;
static MMU_Logic* logic = nullptr;
static CommandRouter* api = nullptr;

void setup() {
    // 1. Create HAL
    hal = new BMCU_Hardware();
    
    // 2. Create Transport
    transport = new UART_Transport();
    transport->Init();
    
    // 3. Create Logic
    logic = new MMU_Logic(hal);
    logic->Init(); // Initializes HAL and Logic
    
    // 4. Init API/Router with transport
    api = new CommandRouter();
    api->Init(logic, transport);
}

void loop() {
    // Run Logic
    if (logic) logic->Run();
    
    // Run API
    if (api) api->Run();
}
