#include <Arduino.h>
#include "Hardware.h"
#include "ControlLogic.h"
#include "CommandRouter.h"

/* DEVELOPMENT STATE: FUNCTIONAL */
void setup()
{
    // Initialize Base Hardware (GPIO, ADC, LED, Timers) - No UART yet
    Hardware::InitBase();
    
    // Initialize Business Logic (Loads settings)
    ControlLogic::Init();
    
    // Check for Boot Mode Switch (Pressure Sensors)
    ControlLogic::BootMode mode = ControlLogic::InitBootCheck();
    bool isKlipper = (mode == ControlLogic::BootMode::Klipper);
    
    // Initialize Communications with specific mode
    Hardware::InitUART(isKlipper);
    CommandRouter::Init(isKlipper);
    
    // Initial LED State
    Hardware::LED_SetBrightness(35); // Default brightness
    Hardware::LED_Show();
}

/* DEVELOPMENT STATE: FUNCTIONAL */
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
