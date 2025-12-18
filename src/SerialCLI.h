#pragma once

#include <stdint.h>
#include <Arduino.h>

namespace SerialCLI {
    
    // Accumulate bytes from serial. Returns true if line is complete.
    bool Accumulate(uint8_t byte);
    
    // Get the prepared line buffer
    char* GetInitBuffer();
    int GetLength();
    void Reset();
    
    // Process a full line of text
    void Parse(char* line);

    // Initialize CLI (print welcome message)
    void Init();

}
