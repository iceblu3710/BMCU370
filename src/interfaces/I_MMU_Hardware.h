#pragma once

#include <stdint.h>

/**
 * @brief Abstract Hardware Abstraction Layer for MMU.
 * 
 * Defines the contract that any hardware platform must implement
 * to support the MMU Logic layer.
 */
class I_MMU_Hardware {
public:
    virtual ~I_MMU_Hardware() {}

    // --- System ---
    virtual void Init() = 0;
    virtual uint64_t GetTimeMS() = 0;
    virtual void DelayMS(uint32_t ms) = 0;
    virtual void WatchdogReset() = 0;

    // --- Motors ---
    /**
     * @brief Set Motor PWM and Direction.
     * 
     * @param lane Lane/Motor index (0-3).
     * @param pwm_val PWM value (-1000 to 1000). Positive = Feed, Negative = Retract.
     */
    virtual void SetMotorPower(int lane, int pwm_val) = 0;

    // --- Sensors ---
    /**
     * @brief Get Filament Pressure Reading.
     * 
     * @param lane Lane index.
     * @return float Voltage (0.0v to 3.3v) representing pressure/tension.
     */
    virtual float GetPressureReading(int lane) = 0;

    /**
     * @brief Get Filament Presence Status.
     * 
     * @param lane Lane index.
     * @return true If filament is detected (Online).
     * @return false If empty.
     */
    virtual bool GetFilamentPresence(int lane) = 0;
    
    /**
     * @brief Update and Get Encoder Angle/Distance.
     * 
     * Depending on implementation, this might poll I2C or return cached value.
     * 
     * @param lane Lane index.
     * @return int32_t Cumulative angle or ticks.
     */
    virtual int32_t GetEncoderValue(int lane) = 0; 
    
    // --- User Feedback ---
    /**
     * @brief Set LED Color.
     * 
     * @param lane Lane index.
     * @param r Red (0-255).
     * @param g Green (0-255).
     * @param b Blue (0-255).
     */
    virtual void SetLED(int lane, uint8_t r, uint8_t g, uint8_t b) = 0;
    
    //=========================================================================
    // PERSISTENT STORAGE (Optional - defaults do nothing)
    //=========================================================================
    
    /**
     * @brief Read settings from non-volatile storage.
     * 
     * Optional method - default implementation returns false (no storage).
     * 
     * @param data   Buffer to read into.
     * @param len    Number of bytes to read.
     * @param offset Address/offset in storage.
     * @return true  Read successful.
     */
    virtual bool ReadNVS(uint8_t* data, uint16_t len, uint32_t offset) { return false; }
    
    /**
     * @brief Write settings to non-volatile storage.
     * 
     * Optional method - default implementation returns false (no storage).
     * 
     * @param data   Buffer to write.
     * @param len    Number of bytes to write.
     * @param offset Address/offset in storage.
     * @return true  Write successful.
     */
    virtual bool WriteNVS(const uint8_t* data, uint16_t len, uint32_t offset) { return false; }
};
