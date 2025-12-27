#pragma once

#include "I_MMU_Transport.h"
#include "Hardware.h"

/**
 * @file UART_Transport.h
 * @brief UART implementation of I_MMU_Transport
 * 
 * Reference implementation for serial communication.
 * Wraps the Hardware::UART_* functions.
 */
class UART_Transport : public I_MMU_Transport {
public:
    UART_Transport() {}
    virtual ~UART_Transport() {}

    void Init() override;
    
    uint16_t Available() override;
    int Read() override;
    uint16_t ReadBytes(uint8_t* buffer, uint16_t len) override;
    uint16_t Write(const uint8_t* data, uint16_t len) override;
    void Flush() override;
    
    bool IsConnected() override;
    bool IsBusy() override;

    // Internal: Called by RX interrupt to buffer incoming bytes
    void OnByteReceived(uint8_t byte);

private:
    // Ring buffer for received bytes (reverted to 1024 for RAM safety)
    static constexpr uint16_t RX_BUFFER_SIZE = 1024;
    volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
    volatile uint16_t rx_head = 0;
    volatile uint16_t rx_tail = 0;
};
