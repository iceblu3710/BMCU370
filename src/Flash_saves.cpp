#include "Flash_saves.h"

/* Global define */
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;
// #define PAGE_WRITE_START_ADDR ((uint32_t)0x08008000) /* Start from 32K */
// #define PAGE_WRITE_END_ADDR ((uint32_t)0x08009000)   /* End at 36K */
#define FLASH_PAGE_SIZE 4096
#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63

/* Global Variable */
uint32_t EraseCounter = 0x0, Address = 0x0;
uint16_t Data = 0xAAAA;
uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;

#define Fadr (0x08020000)
#define Fsize ((((256 * 4)) >> 2))
u32 buf[Fsize];

/*********************************************************************
 * @fn      Flash_Test
 *
 * @brief   Flash Program Test.
 *
 * @return  none
 */
bool Flash_saves(void *buf, uint32_t length, uint32_t address)
{
    uint32_t end_address = address + length;
    uint32_t erase_counter = 0;
    uint32_t address_i = 0;
    uint32_t page_num = (length + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    if (page_num == 0) page_num = 1; // Safety
    uint16_t *data_ptr = (uint16_t *)buf;

    __disable_irq(); // 禁用中断
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

    for (erase_counter = 0; (erase_counter < page_num) && (FLASHStatus == FLASH_COMPLETE); erase_counter++)
    {
        IWDG->CTLR = 0xAAAA; // Reload IWDG
        FLASHStatus = FLASH_ErasePage(address + (FLASH_PAGE_SIZE * erase_counter)); // Erase 4KB

        if (FLASHStatus != FLASH_COMPLETE)
            return false;
    }

    address_i = address;
    while ((address_i < end_address) && (FLASHStatus == FLASH_COMPLETE))
    {
        FLASHStatus = FLASH_ProgramHalfWord(address_i, *data_ptr);
        address_i = address_i + 2;
        data_ptr++;
    }

    FLASH_Lock();
    __enable_irq();
    /*
        address_i = address;
        data_ptr=(uint16_t *)buf;
        while ((address_i < end_address) && (MemoryProgramStatus != FAILED))
        {
            if ((*(__IO uint16_t *)address_i) != *data_ptr)
            {
                MemoryProgramStatus = FAILED;
            }
            address_i += 2;
            data_ptr++;
        }

        if (MemoryProgramStatus == FAILED)
            return false;
        else
            return true;*/
    return true;
}