#pragma once

#include <Arduino.h>

#include "ch32v20x_flash.h"

#define FLASH_PAGE_SIZE 4096

extern bool Flash_saves(void *buf, uint32_t length, uint32_t address);