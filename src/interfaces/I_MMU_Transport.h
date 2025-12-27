#pragma once
#include <stdint.h>

/**
 * @file I_MMU_Transport.h
 * @brief Transport Layer Interface for MMU Communication
 * 
 * Users MUST implement this interface for their specific
 * communication method (UART, USB CDC, CAN, SPI, etc.)
 * 
 * The Protocol layer uses this to send/receive raw bytes.
 */
class I_MMU_Transport {
public:
    virtual ~I_MMU_Transport() {}

    //=========================================================================
    // INITIALIZATION
    //=========================================================================
    
    /**
     * @brief Initialize the transport layer.
     * Configure UART, USB, CAN, etc.
     */
    virtual void Init() = 0;

    //=========================================================================
    // DATA TRANSFER
    //=========================================================================
    
    /**
     * @brief Check if data is available to read.
     * @return Number of bytes available, or 0 if none.
     */
    virtual uint16_t Available() = 0;
    
    /**
     * @brief Read a single byte from the host.
     * @return Byte read, or -1 if none available.
     */
    virtual int Read() = 0;
    
    /**
     * @brief Read multiple bytes from the host.
     * @param buffer Buffer to read into.
     * @param len    Maximum bytes to read.
     * @return Actual number of bytes read.
     */
    virtual uint16_t ReadBytes(uint8_t* buffer, uint16_t len) = 0;
    
    /**
     * @brief Write bytes to the host.
     * @param data Data buffer to send.
     * @param len  Number of bytes to send.
     * @return Number of bytes actually written.
     */
    virtual uint16_t Write(const uint8_t* data, uint16_t len) = 0;
    
    /**
     * @brief Flush any buffered output data.
     */
    virtual void Flush() = 0;

    //=========================================================================
    // CONNECTION STATUS
    //=========================================================================
    
    /**
     * @brief Check if transport is connected and ready.
     * @return true if host connection is active.
     */
    virtual bool IsConnected() = 0;

    /**
     * @brief Check if transport is currently transmitting data.
     * @return true if transmission is in progress.
     */
    virtual bool IsBusy() = 0;
};
