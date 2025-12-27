/**
 * @file Flash_saves.h
 * @brief Non-volatile Flash storage driver API for CH32V203 MCU.
 * 
 * @details
 * DEVELOPMENT STATE: FUNCTIONAL - PROVEN STABLE - DO NOT MODIFY
 * 
 * Provides a simple API to persist data to internal Flash memory.
 * Used by MMU_Logic to store filament information, calibration data,
 * and boot configuration.
 * 
 * @warning Flash writes are blocking. Use with smart save timing
 *          (see KlipperCLI::IsSerialIdle) to avoid serial timeouts.
 * 
 * @author BMCU370 Development Team
 * @version 1.0.0
 * @date 2025-12-27
 */
#pragma once

#include <Arduino.h>
#include "ch32v20x_flash.h"

/// CH32V203 Flash page size (4KB)
#define FLASH_PAGE_SIZE 4096

/**
 * @brief Save a data buffer to internal Flash memory.
 * 
 * @param buf     Pointer to source data buffer.
 * @param length  Length of data in bytes.
 * @param address Destination Flash address (page-aligned recommended).
 * 
 * @return true   Flash write completed successfully.
 * @return false  Flash write failed.
 * 
 * @note This function is BLOCKING (~20-50ms per 4KB page).
 * @warning Ensure serial is idle before calling to prevent communication timeouts.
 */
extern bool Flash_saves(void *buf, uint32_t length, uint32_t address);