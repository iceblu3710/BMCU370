#include <Arduino.h>
#include "Hardware.h"
#include "ControlLogic.h"
#include "CommandRouter.h"

void setup()
{
    // Initialize Hardware (Clocks, GPIO, UART, Timer, ADC, LED)
    Hardware::Init();
    
    // Initialize Business Logic (Settings, State, Motors)
    ControlLogic::Init();
    
    // Initialize Communications (Protocol, Callbacks)
    CommandRouter::Init();
    
    // Initial LED State
    Hardware::LED_SetBrightness(35); // Default brightness
    Hardware::LED_Show();
}

void loop()
{
    while (1)
    {
        // Process Communications
        CommandRouter::Run();
        
        // Run Control Logic (Motors, State Machine)
        ControlLogic::Run();
    }
}
