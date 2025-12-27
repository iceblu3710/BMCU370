#pragma once

#include <stdint.h>
#include "MMU_Defs.h"

#include <string.h>

// --- Shared Data Structures ---

struct FilamentInfo {
    char ID[8];
    char name[20];
    uint8_t color_R;
    uint8_t color_G;
    uint8_t color_B;
    uint8_t color_A;
    uint16_t temperature_min;
    uint16_t temperature_max;
    
    // Default constructor - zero initialize all fields
    FilamentInfo() : color_R(0), color_G(0), color_B(0), color_A(0), 
                     temperature_min(0), temperature_max(0) {
        memset(ID, 0, sizeof(ID));
        memset(name, 0, sizeof(name));
    }
    
    // Helper accessors
    void SetID(const char* new_id) {
        strncpy(ID, new_id, sizeof(ID)-1);
        ID[sizeof(ID)-1] = 0;
    }
    void SetName(const char* new_name) {
        strncpy(name, new_name, sizeof(name)-1);
        name[sizeof(name)-1] = 0;
    }
};

struct FilamentState : public FilamentInfo
{
    float meters = 0;
    uint16_t pressure = 0;
    AMS_filament_status status = AMS_filament_status::offline;
    AMS_filament_motion motion_set = AMS_filament_motion::idle;
    uint32_t meters_virtual_count = 0;
};

// UnitState class removed. Use MMU_Logic accessors instead.
