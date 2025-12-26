#pragma once

#include <stdint.h>
#include "BambuBusProtocol.h" // For AMS_filament_status/motion enums

#pragma pack(push, 1)

/*
* DEVELOPMENT STATE: FUNCTIONAL
* DO NOT MODIFY under any circumstances.
*/
struct FilamentInfo {
    char ID[8];
    uint8_t color_R;
    uint8_t color_G;
    uint8_t color_B;
    uint8_t color_A;
    int16_t temperature_min;
    int16_t temperature_max;
    char name[20];

    // Defaults constructor
    FilamentInfo() : 
        color_R(0xFF), color_G(0xFF), color_B(0xFF), color_A(0xFF),
        temperature_min(220), temperature_max(240)
    {
         // Manual string init
         const char* def_id = "GFG00";
         for(int i=0;i<8;i++) ID[i] = (i < 5) ? def_id[i] : 0;
         
         const char* def_name = "PETG";
         for(int i=0;i<20;i++) name[i] = (i < 4) ? def_name[i] : 0;
    }
    
    // Helper to set ID safely
    void SetID(const char* new_id);
    void SetName(const char* new_name);
};

/*
* DEVELOPMENT STATE: FUNCTIONAL
* DO NOT MODIFY under any circumstances.
*/
struct FilamentState : public FilamentInfo {
    // Inherits ID, Color, Temp, Name from FilamentInfo
    
    float meters;
    uint64_t meters_virtual_count;
    AMS_filament_status status;
    AMS_filament_motion motion_set;
    uint16_t pressure;
    
    // C++11 Member Init
    FilamentState() : FilamentInfo(),
        meters(0), meters_virtual_count(0),
        status(AMS_filament_status::online),
        motion_set(AMS_filament_motion::idle),
        pressure(0xFFFF)
    {
         // FilamentInfo ctor handles basic fields
    }
};
#pragma pack(pop)

/*
* DEVELOPMENT STATE: FUNCTIONAL
* DO NOT MODIFY under any circumstances.
*/
class UnitState {
public:
    static FilamentState& GetFilament(int index);
    static int GetCurrentFilamentIndex();
    static void SetCurrentFilamentIndex(int index);
    static uint8_t GetFilamentUseFlag();
    static void SetFilamentUseFlag(uint8_t flag);
    
    // BambuBus Address (AMS Identifier, e.g., 0x0700)
    static void SetBusAddress(uint16_t addr);
    static uint16_t GetBusAddress();
    static const char* GetModel();
    static const char* GetVersion();
    static const char* GetSerialNumber();
};
