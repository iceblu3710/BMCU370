#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "ch32v20x.h"

// Hardware Abstraction Layer

namespace Hardware {

    void Init();

    // Time
    void DelayUS(uint32_t us);
    void DelayMS(uint32_t ms);
    uint64_t GetTime(); // Returns time in ms or similar, based on time64.h

    // UART
    void UART_Init();
    void UART_SetRxCallback(void (*callback)(uint8_t));
    void UART_Send(const uint8_t *data, uint16_t length);

    // ADC
    void ADC_Init();
    float* ADC_GetValues(); // Returns pointer to 8 floats

    // PWM / Motor
    void PWM_Init();
    void PWM_Set(uint8_t channel, int pwm_value);

    // LED
    void LED_Init();
    void LED_SetColor(uint8_t channel, int led_idx, uint8_t r, uint8_t g, uint8_t b);
    void LED_Show();
    void LED_SetBrightness(uint8_t brightness);

    // Watchdog
    void Watchdog_Disable();
    
    // System
    void System_Init(); // Configures RCC, GPIO remap, etc.
}
