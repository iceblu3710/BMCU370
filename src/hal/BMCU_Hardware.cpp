#include "BMCU_Hardware.h"
#include "Hardware.h"
#include "AS5600.h"

// External reference to AS5600 object in ControlLogic? 
// Or should we move AS5600 instance here?
// Ideally AS5600 should be managed by HAL.
// But ControlLogic currently owns "MC_AS5600".
// For the Refactor, we should probably move MC_AS5600 to BMCU_Hardware or make it a member.
// Current ControlLogic.cpp has: static AS5600_soft_IIC MC_AS5600;
// We cannot easily access it if it's static in .cpp.
// Capability gap: We need to instantiate AS5600 here.
// We need to copy the pin definitions.

static AS5600_soft_IIC HAL_AS5600;
static uint32_t HAL_AS5600_SCL[] = {PB15, PB14, PB13, PB12};
static uint32_t HAL_AS5600_SDA[] = {PD0, PC15, PC14, PC13};

BMCU_Hardware::BMCU_Hardware() {
}

void BMCU_Hardware::Init() {
    Hardware::InitBase();
    Hardware::ADC_Init();
    Hardware::PWM_Init();
    Hardware::LED_Init();
    
    HAL_AS5600.init(HAL_AS5600_SCL, HAL_AS5600_SDA, 4);
}

uint64_t BMCU_Hardware::GetTimeMS() {
    return Hardware::GetTime();
}

void BMCU_Hardware::DelayMS(uint32_t ms) {
    Hardware::DelayMS(ms);
}

void BMCU_Hardware::WatchdogReset() {
    // Hardware::Watchdog_Reset(); // If implemented
}

void BMCU_Hardware::SetMotorPower(int lane, int pwm_val) {
    Hardware::PWM_Set(lane, pwm_val);
}

float BMCU_Hardware::GetPressureReading(int lane) {
    float *vals = Hardware::ADC_GetValues();
    if (!vals) return 0.0f;
    // Map: [0]=CH4.. [6]=CH1 based on ControlLogic read?
    // ControlLogic:
    // PULL_raw[3] = data[0]; (CH4)
    // PULL_raw[2] = data[2]; (CH3)
    // PULL_raw[1] = data[4]; (CH2)
    // PULL_raw[0] = data[6]; (CH1)
    
    // So index = (3 - lane) * 2 ?
    // If lane=0 -> data[6]. (3-0)*2 = 6. Correct.
    // If lane=3 -> data[0]. (3-3)*2 = 0. Correct.
    int idx = (3 - lane) * 2;
    if (idx < 0 || idx > 7) return 0.0f;
    return vals[idx];
}

bool BMCU_Hardware::GetFilamentPresence(int lane) {
    float *vals = Hardware::ADC_GetValues();
    if (!vals) return false;
    // ONLINE_raw[3] = data[1];
    // ONLINE_raw[0] = data[7];
    // Idx = (3-lane)*2 + 1.
    int idx = (3 - lane) * 2 + 1;
    if (idx < 0 || idx > 7) return false;
    
    float v = vals[idx];
    // Threshold from ControlLogic?
    // "if (MC_ONLINE_key_stu_raw[i] > 1.65f)" -> Present (1)
    // Logic was:
    // (MC_ONLINE_key_stu_raw[i] > 1.65f) -> 1
    // (MC_ONLINE_key_stu_raw[i] < 1.55f) -> 0
    // Using 1.6V threshold
    return (v > 1.6f);
}

int32_t BMCU_Hardware::GetEncoderValue(int lane) {
    HAL_AS5600.updata_angle(); // This updates ALL. Maybe inefficient to do per lane call?
    // But Interface implies per-lane query.
    // We could optimize by having an Update() method in HAL called by loop.
    // But for now, simple implementation.
    return HAL_AS5600.raw_angle[lane]; // This is raw 0-4096.
    // Wait, ControlLogic calculated "accumulated" distance.
    // The Interface returns "Value" (Angle).
    // The Logic layer calculates Distance.
    // So returning raw angle is OK.
}

void BMCU_Hardware::SetLED(int lane, uint8_t r, uint8_t g, uint8_t b) {
    Hardware::LED_SetColor(lane, 0, r, g, b);
    Hardware::LED_Show();
}
