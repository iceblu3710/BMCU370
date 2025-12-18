#pragma once

#include <stdint.h>
#include "BambuBusProtocol.h"
#include "UnitState.h"

namespace ControlLogic {

    void Init();
    void Run();
    
    // Connectivity
    void UpdateConnectivity(bool online);
    
    // --- Logic / Action Methods (High Level) ---
    // These methods act on the Unit State or Hardware directly, independent of Protocol
    
    // Corresponds to MotionShort (0x03)
    void ProcessMotionShortLogic(uint8_t ams_num, uint8_t status_flags, uint8_t read_num, uint8_t motion_flag);
    
    // Corresponds to MotionLong (0x04) - TODO: Break down further if payload is complex
    void ProcessMotionLongLogic(uint8_t* payload, uint16_t length);
    
    // Corresponds to SetFilamentInfo (0x08 / 0x218)
    void SetFilamentInfoAction(int id, const FilamentInfo& info);
    
    // --- Protocol Wrappers (Parse buffers and call Logic) ---
    void ProcessMotionShort(uint8_t* buffer, uint16_t length);
    void ProcessMotionLong(uint8_t* buffer, uint16_t length);
    void ProcessOnlineDetect(uint8_t* buffer, uint16_t length);
    void ProcessREQx6(uint8_t* buffer, uint16_t length);
    void ProcessSetFilamentInfo(uint8_t* buffer, uint16_t length);
    void ProcessHeartbeat(uint8_t* buffer, uint16_t length);
    
    // Command Processing (Long)
    void ProcessLongPacket(struct long_packge_data &data);

    // Manual Operations
    void StartLoadFilament(int tray, int length_mm = -1);
    void StartUnloadFilament(int tray, int length_mm = -1);
    
    // State Accessors
    uint16_t GetDeviceType();

    // --- Persistence ---
    void SaveSettings();
    void SetNeedToSave();
}

