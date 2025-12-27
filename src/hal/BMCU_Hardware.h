#pragma once

#include "I_MMU_Hardware.h"

class BMCU_Hardware : public I_MMU_Hardware {
public:
    BMCU_Hardware();
    virtual ~BMCU_Hardware() {}

    // --- System ---
    void Init() override;
    uint64_t GetTimeMS() override;
    void DelayMS(uint32_t ms) override;
    void WatchdogReset() override;

    // --- Motors ---
    void SetMotorPower(int lane, int pwm_val) override;

    // --- Sensors ---
    float GetPressureReading(int lane) override;
    bool GetFilamentPresence(int lane) override;
    int32_t GetEncoderValue(int lane) override; 

    // --- User Feedback ---
    void SetLED(int lane, uint8_t r, uint8_t g, uint8_t b) override;
};
