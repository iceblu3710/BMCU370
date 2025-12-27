/**
 * @file UART_Transport.cpp
 * @brief UART implementation of I_MMU_Transport
 */
#include "UART_Transport.h"

// Global instance pointer for interrupt callback
static UART_Transport* g_transport_instance = nullptr;

// Callback for Hardware RX interrupt
static void uart_rx_callback(uint8_t byte) {
    if (g_transport_instance) {
        g_transport_instance->OnByteReceived(byte);
    }
}

void UART_Transport::Init() {
    g_transport_instance = this;
    Hardware::InitUART(true); // Klipper mode UART
    Hardware::UART_SetRxCallback(uart_rx_callback);
}

uint16_t UART_Transport::Available() {
    uint16_t head = rx_head;
    uint16_t tail = rx_tail;
    if (head >= tail) {
        return head - tail;
    } else {
        return RX_BUFFER_SIZE - tail + head;
    }
}

int UART_Transport::Read() {
    if (rx_head == rx_tail) {
        return -1; // No data
    }
    uint8_t byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return byte;
}

uint16_t UART_Transport::ReadBytes(uint8_t* buffer, uint16_t len) {
    uint16_t count = 0;
    while (count < len && rx_head != rx_tail) {
        buffer[count++] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }
    return count;
}

uint16_t UART_Transport::Write(const uint8_t* data, uint16_t len) {
    Hardware::UART_Send(data, len);
    return len;
}

void UART_Transport::Flush() {
    // Hardware UART is synchronous, nothing to flush
}

bool UART_Transport::IsConnected() {
    // UART is always "connected" when initialized
    return true;
}

bool UART_Transport::IsBusy() {
    return Hardware::UART_IsBusy();
}

void UART_Transport::OnByteReceived(uint8_t byte) {
    uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    if (next_head != rx_tail) {
        rx_buffer[rx_head] = byte;
        rx_head = next_head;
    }
    // If buffer full, drop byte
}
