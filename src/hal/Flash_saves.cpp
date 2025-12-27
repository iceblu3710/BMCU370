/**
 * @file Flash_saves.cpp
 * @brief Non-volatile Flash storage driver for CH32V203 MCU.
 * 
 * @details
 * DEVELOPMENT STATE: FUNCTIONAL - PROVEN STABLE - DO NOT MODIFY
 * 
 * This driver provides persistent storage by writing data directly to
 * the MCU's internal Flash memory. Used for storing filament information,
 * calibration data, and boot configuration.
 * 
 * Key Design Decisions:
 * - **IRQ-safe**: Does NOT disable interrupts during Flash operations.
 *   This prevents serial communication timeouts at the cost of slight
 *   Flash operation timing sensitivity. Validated stable in practice.
 * - **Watchdog-safe**: Reloads IWDG during long erase/program cycles.
 * - **Page-aligned writes**: Erases full 4KB pages before programming.
 * 
 * Usage Notes:
 * - Flash writes are blocking (~20-50ms per page).
 * - Caller should ensure serial is idle before calling (see KlipperCLI::IsSerialIdle).
 * - Maximum recommended write frequency: once per 5 seconds (debounced in MMU_Logic).
 * 
 * @warning Frequent Flash writes reduce Flash lifespan. The debounce timer
 *          in MMU_Logic ensures writes are batched appropriately.
 * 
 * @author BMCU370 Development Team
 * @version 1.0.0
 * @date 2025-12-27
 */
#include "Flash_saves.h"

/* Global define */
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define FLASH_PAGE_SIZE 4096                          ///< CH32V203 Flash page size (4KB)
#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63

/* Global Variable */
uint32_t EraseCounter = 0x0, Address = 0x0;
uint16_t Data = 0xAAAA;
uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;

// Redundant global buffer removed to save ~1KB RAM
// #define Fadr (0x08020000)
// #define Fsize ((((256 * 4)) >> 2))
// u32 buf[Fsize];

/**
 * @brief Save a buffer to internal Flash memory.
 * 
 * Erases the required number of 4KB pages and programs the data.
 * Reloads the Independent Watchdog (IWDG) during the process to prevent
 * system reset during long operations.
 * 
 * @note This function is BLOCKING and takes ~20-50ms per page.
 *       Ensure serial communication is idle before calling.
 * 
 * @param buf     Pointer to the source data buffer.
 * @param length  Length of data in bytes.
 * @param address Destination Flash address (should be page-aligned for best results).
 * 
 * @return true   Flash write completed successfully.
 * @return false  Flash write failed (check FLASHStatus for details).
 * 
 * @warning Do not call during active serial communication - use debounce timer.
 */
bool Flash_saves(void *buf, uint32_t length, uint32_t address)
{
    uint32_t end_address = address + length;
    uint32_t erase_counter = 0;
    uint32_t address_i = 0;
    uint32_t page_num = (length + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    if (page_num == 0) page_num = 1; // Safety
    uint16_t *data_ptr = (uint16_t *)buf;

    // __disable_irq(); // Removed to prevent serial corruption during Flash erase/write
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

    for (erase_counter = 0; (erase_counter < page_num) && (FLASHStatus == FLASH_COMPLETE); erase_counter++)
    {
        IWDG->CTLR = 0xAAAA; // Reload IWDG
        FLASHStatus = FLASH_ErasePage(address + (FLASH_PAGE_SIZE * erase_counter)); // Erase 4KB

        if (FLASHStatus != FLASH_COMPLETE)
        {
            FLASH_Lock();
            __enable_irq();
            return false;
        }
    }

    address_i = address;
    while ((address_i < end_address) && (FLASHStatus == FLASH_COMPLETE))
    {
        IWDG->CTLR = 0xAAAA; // Reload IWDG during long writes
        FLASHStatus = FLASH_ProgramHalfWord(address_i, *data_ptr);
        address_i = address_i + 2;
        data_ptr++;
    }

    FLASH_Lock();
    // __enable_irq(); // Removed to prevent serial corruption
    return (FLASHStatus == FLASH_COMPLETE);
}