#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "ch32v20x.h"

// Motor Direction Configuration
// Set to true to invert the motor direction for the specific channel
#define MOTOR_INVERT_CH1 false
#define MOTOR_INVERT_CH2 false
#define MOTOR_INVERT_CH3 false
#define MOTOR_INVERT_CH4 true

// Motor Speed Configuration
// #define MOTOR_SPEED_SEND 50
// #define MOTOR_SPEED_AMS_LITE_SEND 30
// #define MOTOR_SPEED_SLOW_SEND 3
// #define MOTOR_SPEED_PULL 50

// PID Inversion Configuration
#define MOTOR_PID_INVERT_CH1 true
#define MOTOR_PID_INVERT_CH2 true
#define MOTOR_PID_INVERT_CH3 true
#define MOTOR_PID_INVERT_CH4 false

// Device Info
#ifndef DEVICE_MODEL
#define DEVICE_MODEL "BMCU-370c v2.2 Board"
#endif
#ifndef DEVICE_VERSION
#define DEVICE_VERSION "v0-20"
#endif
#ifndef DEVICE_SERIAL
#define DEVICE_SERIAL "SN123456"
#endif

// Hardware Abstraction Layer

/*
* DEVELOPMENT STATE: FUNCTIONAL
* The entire hardware layer (LEDs, motors, pressure, encoder, etc) is PROVEN FUNCTIONAL.
* DO NOT MODIFY under any circumstances.
*/
namespace Hardware {

    void InitBase();
    /* DEVELOPMENT STATE: TESTING */
    void InitUART(bool isKlipper);

    // Time
    void DelayUS(uint32_t us);
    void DelayMS(uint32_t ms);
    uint64_t GetTime(); // Returns time in ms or similar, based on time64.h
    
    // UART
    // void UART_Init(); // Removed, use InitUART(bool)
    void UART_SetRxCallback(void (*callback)(uint8_t));
    /* DEVELOPMENT STATE: TESTING */
    void UART_Send(const uint8_t *data, uint16_t length);
    void UART_SendByte(uint8_t data);
    bool UART_IsBusy();

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
